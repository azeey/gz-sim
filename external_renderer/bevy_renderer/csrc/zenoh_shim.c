#include <zenoh.h>

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef void (*zshim_sample_cb_t)(const uint8_t *data, size_t len, bool is_shm, void *user_ctx);

typedef struct zshim_session {
    z_owned_session_t session;
    z_owned_memory_layout_t shm_layout;
    z_owned_shm_provider_t shm_provider;
    bool has_shm;
    pthread_mutex_t shm_mtx;
} zshim_session_t;

typedef struct zshim_publisher {
    zshim_session_t *sess;
    z_owned_publisher_t pub;
} zshim_publisher_t;

typedef struct zshim_subscriber {
    z_owned_subscriber_t sub;
    zshim_sample_cb_t cb;
    void *user_ctx;
} zshim_subscriber_t;

static void on_sample_trampoline(z_loaned_sample_t *sample, void *ctx) {
    zshim_subscriber_t *s = (zshim_subscriber_t *)ctx;
    if (!s || !s->cb) return;

    const z_loaned_bytes_t *payload = z_sample_payload(sample);
    const z_loaned_shm_t *shm = NULL;
    if (z_bytes_as_loaned_shm(payload, &shm) == Z_OK && shm != NULL) {
        s->cb(z_shm_data(shm), z_shm_len(shm), true, s->user_ctx);
        return;
    }

    z_owned_slice_t slice;
    if (z_bytes_to_slice(payload, &slice) == Z_OK) {
        const z_loaned_slice_t *loaned = z_loan(slice);
        s->cb(z_slice_data(loaned), z_slice_len(loaned), false, s->user_ctx);
        z_drop(z_move(slice));
    }
}

zshim_session_t *zshim_open(const char *endpoint, bool listen_mode, size_t shm_pool_bytes) {
    zshim_session_t *s = (zshim_session_t *)calloc(1, sizeof(zshim_session_t));
    if (!s) return NULL;
    pthread_mutex_init(&s->shm_mtx, NULL);

    z_owned_config_t cfg;
    z_config_default(&cfg);
    zc_config_insert_json5(z_loan_mut(cfg), "mode", "\"peer\"");
    zc_config_insert_json5(z_loan_mut(cfg), "scouting/multicast/enabled", "false");
    zc_config_insert_json5(z_loan_mut(cfg), "transport/shared_memory/enabled", "true");

    char ep_json[512];
    snprintf(ep_json, sizeof(ep_json), "[\"%s\"]", endpoint);
    if (listen_mode) {
        zc_config_insert_json5(z_loan_mut(cfg), "listen/endpoints", ep_json);
    } else {
        zc_config_insert_json5(z_loan_mut(cfg), "connect/endpoints", ep_json);
    }

    if (z_open(&s->session, z_move(cfg), NULL) < 0) {
        fprintf(stderr, "[zshim] z_open failed for %s\n", endpoint);
        pthread_mutex_destroy(&s->shm_mtx);
        free(s);
        return NULL;
    }

    if (shm_pool_bytes > 0) {
        z_alloc_alignment_t align = {3};  // 8-byte aligned
        if (z_memory_layout_new(&s->shm_layout, shm_pool_bytes, align) == Z_OK) {
            if (z_posix_shm_provider_with_layout_new(&s->shm_provider, z_loan(s->shm_layout)) == Z_OK) {
                s->has_shm = true;
            } else {
                z_drop(z_move(s->shm_layout));
            }
        }
    }
    return s;
}

void zshim_close(zshim_session_t *s) {
    if (!s) return;
    if (s->has_shm) {
        z_drop(z_move(s->shm_provider));
        z_drop(z_move(s->shm_layout));
        s->has_shm = false;
    }
    z_drop(z_move(s->session));
    pthread_mutex_destroy(&s->shm_mtx);
    free(s);
}

zshim_publisher_t *zshim_declare_publisher(zshim_session_t *s, const char *key_expr) {
    if (!s || !key_expr) return NULL;
    zshim_publisher_t *p = (zshim_publisher_t *)calloc(1, sizeof(zshim_publisher_t));
    if (!p) return NULL;
    p->sess = s;

    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str(&ke, key_expr);
    if (z_declare_publisher(z_loan(s->session), &p->pub, z_loan(ke), NULL) < 0) {
        fprintf(stderr, "[zshim] declare_publisher failed for %s\n", key_expr);
        free(p);
        return NULL;
    }
    return p;
}

bool zshim_publish(zshim_publisher_t *p, const uint8_t *data, size_t len, bool prefer_shm) {
    if (!p || !p->sess) return false;
    z_owned_bytes_t payload;
    bool used_shm = false;

    if (prefer_shm && p->sess->has_shm && len > 0) {
        pthread_mutex_lock(&p->sess->shm_mtx);
        z_buf_layout_alloc_result_t res;
        z_shm_provider_alloc_gc_defrag(&res, z_loan(p->sess->shm_provider), len);
        if (res.status == ZC_BUF_LAYOUT_ALLOC_STATUS_OK) {
            memcpy(z_shm_mut_data_mut(z_loan_mut(res.buf)), data, len);
            z_bytes_from_shm_mut(&payload, z_move(res.buf));
            used_shm = true;
        }
        pthread_mutex_unlock(&p->sess->shm_mtx);
    }

    if (!used_shm) {
        z_bytes_copy_from_buf(&payload, data, len);
    }
    return z_publisher_put(z_loan(p->pub), z_move(payload), NULL) == Z_OK;
}

void zshim_undeclare_publisher(zshim_publisher_t *p) {
    if (!p) return;
    z_drop(z_move(p->pub));
    free(p);
}

zshim_subscriber_t *zshim_declare_subscriber(
    zshim_session_t *s,
    const char *key_expr,
    zshim_sample_cb_t cb,
    void *user_ctx) {
    if (!s || !key_expr || !cb) return NULL;
    zshim_subscriber_t *sub = (zshim_subscriber_t *)calloc(1, sizeof(zshim_subscriber_t));
    if (!sub) return NULL;
    sub->cb = cb;
    sub->user_ctx = user_ctx;

    z_owned_closure_sample_t closure;
    z_closure(&closure, on_sample_trampoline, NULL, sub);

    z_view_keyexpr_t ke;
    z_view_keyexpr_from_str(&ke, key_expr);
    if (z_declare_subscriber(z_loan(s->session), &sub->sub, z_loan(ke), z_move(closure), NULL) < 0) {
        fprintf(stderr, "[zshim] declare_subscriber failed for %s\n", key_expr);
        free(sub);
        return NULL;
    }
    return sub;
}

void zshim_undeclare_subscriber(zshim_subscriber_t *sub) {
    if (!sub) return;
    z_drop(z_move(sub->sub));
    free(sub);
}

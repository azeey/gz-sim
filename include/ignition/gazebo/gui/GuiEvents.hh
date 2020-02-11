/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef IGNITION_GAZEBO_GUI_GUIEVENTS_HH_
#define IGNITION_GAZEBO_GUI_GUIEVENTS_HH_

#include <QEvent>
#include <set>
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/config.hh"

namespace ignition
{
namespace gazebo
{
namespace gui {
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
/// \brief Namespace for all events. Refer to the EventManager class for
/// more information about events.
namespace events
{
  /// \brief Event that notifies when new entities have been selected.
  class EntitiesSelected : public QEvent
  {
    /// \brief Constructor
    /// \param[in] _entities All the selected entities
    /// \param[in] _fromUser True if the event was directly generated by the
    /// user, false in case it's been propagated through a different mechanism.
    public: explicit EntitiesSelected(const std::set<Entity> &_entities,
        bool _fromUser = false)
        : QEvent(Type), entities(_entities), fromUser(_fromUser)
    {
    }

    /// \brief Get the data sent with the event.
    /// \return The entities being selected.
    public: std::set<Entity> Data() const
    {
      return this->entities;
    }

    /// \brief Get whether the event was generated by the user.
    /// \return True for the user.
    public: bool FromUser() const
    {
      return this->fromUser;
    }

    /// \brief Unique type for this event.
    static const QEvent::Type Type = QEvent::Type(QEvent::User + 1);

    /// \brief The selected entities.
    private: std::set<Entity> entities;

    /// \brief Whether the event was generated by the user,
    private: bool fromUser{false};
  };

  /// \brief Event that notifies when all entities have been deselected.
  class DeselectAllEntities : public QEvent
  {
    /// \brief Constructor
    /// \param[in] _fromUser True if the event was directly generated by the
    /// user, false in case it's been propagated through a different mechanism.
    public: explicit DeselectAllEntities(bool _fromUser = false)
        : QEvent(Type), fromUser(_fromUser)
    {
    }

    /// \brief Get whether the event was generated by the user.
    /// \return True for the user.
    public: bool FromUser() const
    {
      return this->fromUser;
    }

    /// \brief Unique type for this event.
    static const QEvent::Type Type = QEvent::Type(QEvent::User + 2);

    /// \brief Whether the event was generated by the user,
    private: bool fromUser{false};
  };
}  // namespace events
}
}  // namespace gui
}  // namespace gazebo
}  // namespace ignition

#endif  // IGNITION_GAZEBO_GUI_GUIEVENTS_HH_

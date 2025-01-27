# ROS System Overview

Generated at: 2025-01-27 06:37:11

## Nodes

### /temperature_publisher

**Namespace**: /

Publishers:
- /rosout
- /sensor/temperature

Subscribers:
*No subscribers*

### /environment_builder

**Namespace**: /

Publishers:
- /rosout
- /environment/data

Subscribers:
- /data/processed

### /conflicting_temperature_publisher

**Namespace**: /

Publishers:
- /rosout
- /sensor/temperature

Subscribers:
*No subscribers*

### /rosout

**Namespace**: /

Publishers:
- /rosout_agg

Subscribers:
- /rosout

### /humidity_publisher

**Namespace**: /

Publishers:
- /rosout
- /sensor/humidity

Subscribers:
*No subscribers*

### /move_to_goal

**Namespace**: /

Publishers:
- /rosout
- /move_to_goal/status
- /move_to_goal/result
- /move_to_goal/feedback
- /cmd_vel

Subscribers:
- /environment/data

### /move_robot

**Namespace**: /

Publishers:
- /rosout
- /robot/status

Subscribers:
- /cmd_vel

### /visualize_data

**Namespace**: /

Publishers:
- /rosout

Subscribers:
- /data/processed

### /data_processor

**Namespace**: /

Publishers:
- /rosout
- /data/processed

Subscribers:
- /data/filtered

### /delayed_response

**Namespace**: /

Publishers:
- /rosout

Subscribers:
*No subscribers*

### /command_listener

**Namespace**: /

Publishers:
- /rosout
- /cmd_vel

Subscribers:
*No subscribers*

### /data_filter

**Namespace**: /

Publishers:
- /rosout
- /data/filtered

Subscribers:
- /sensor/humidity
- /sensor/temperature


## Topics

### /rosout_agg

**Type**: rosgraph_msgs/Log

Publishers:
- /rosout

Subscribers:
*No subscribers*

### /rosout

**Type**: rosgraph_msgs/Log

Publishers:
- /humidity_publisher
- /temperature_publisher
- /move_robot
- /conflicting_temperature_publisher
- /visualize_data
- /data_filter
- /data_processor
- /environment_builder
- /move_to_goal
- /delayed_response
- /command_listener

Subscribers:
- /rosout

### /sensor/humidity

**Type**: std_msgs/Float64

Publishers:
- /humidity_publisher

Subscribers:
- /data_filter

### /sensor/temperature

**Type**: std_msgs/Float64

Publishers:
- /temperature_publisher
- /conflicting_temperature_publisher

Subscribers:
- /data_filter

### /robot/status

**Type**: std_msgs/String

Publishers:
- /move_robot

Subscribers:
*No subscribers*

### /data/filtered

**Type**: data_processing/FilteredData

Publishers:
- /data_filter

Subscribers:
- /data_processor

### /data/processed

**Type**: std_msgs/String

Publishers:
- /data_processor

Subscribers:
- /visualize_data
- /environment_builder

### /environment/data

**Type**: environment_integration/EnvironmentData

Publishers:
- /environment_builder

Subscribers:
- /move_to_goal

### /move_to_goal/status

**Type**: actionlib_msgs/GoalStatusArray

Publishers:
- /move_to_goal

Subscribers:
*No subscribers*

### /move_to_goal/result

**Type**: action_server/MoveToGoalActionResult

Publishers:
- /move_to_goal

Subscribers:
*No subscribers*

### /move_to_goal/feedback

**Type**: action_server/MoveToGoalActionFeedback

Publishers:
- /move_to_goal

Subscribers:
*No subscribers*

### /cmd_vel

**Type**: geometry_msgs/Twist

Publishers:
- /move_to_goal
- /command_listener

Subscribers:
- /move_robot


## System Graph

```mermaid
graph TD
    %% Style definitions
    classDef node fill:#e1f5fe,stroke:#01579b,stroke-width:2px;
    classDef topic fill:#f3e5f5,stroke:#4a148c,stroke-width:2px;

    %% Nodes and Topics
    _temperature_publisher["/temperature_publisher"]:::node
    _environment_builder["/environment_builder"]:::node
    _conflicting_temperature_publisher["/conflicting_temperature_publisher"]:::node
    _rosout["/rosout"]:::node
    _humidity_publisher["/humidity_publisher"]:::node
    _move_to_goal["/move_to_goal"]:::node
    _move_robot["/move_robot"]:::node
    _visualize_data["/visualize_data"]:::node
    _data_processor["/data_processor"]:::node
    _delayed_response["/delayed_response"]:::node
    _command_listener["/command_listener"]:::node
    _data_filter["/data_filter"]:::node
    _rosout_agg[["/rosout_agg"]]:::topic
    _rosout[["/rosout"]]:::topic
    _sensor_humidity[["/sensor/humidity"]]:::topic
    _sensor_temperature[["/sensor/temperature"]]:::topic
    _robot_status[["/robot/status"]]:::topic
    _data_filtered[["/data/filtered"]]:::topic
    _data_processed[["/data/processed"]]:::topic
    _environment_data[["/environment/data"]]:::topic
    _move_to_goal_status[["/move_to_goal/status"]]:::topic
    _move_to_goal_result[["/move_to_goal/result"]]:::topic
    _move_to_goal_feedback[["/move_to_goal/feedback"]]:::topic
    _cmd_vel[["/cmd_vel"]]:::topic

    %% Connections
    _rosout --> _rosout_agg
    _humidity_publisher --> _rosout
    _temperature_publisher --> _rosout
    _move_robot --> _rosout
    _conflicting_temperature_publisher --> _rosout
    _visualize_data --> _rosout
    _data_filter --> _rosout
    _data_processor --> _rosout
    _environment_builder --> _rosout
    _move_to_goal --> _rosout
    _delayed_response --> _rosout
    _command_listener --> _rosout
    _rosout --> _rosout
    _humidity_publisher --> _sensor_humidity
    _sensor_humidity --> _data_filter
    _temperature_publisher --> _sensor_temperature
    _conflicting_temperature_publisher --> _sensor_temperature
    _sensor_temperature --> _data_filter
    _move_robot --> _robot_status
    _data_filter --> _data_filtered
    _data_filtered --> _data_processor
    _data_processor --> _data_processed
    _data_processed --> _visualize_data
    _data_processed --> _environment_builder
    _environment_builder --> _environment_data
    _environment_data --> _move_to_goal
    _move_to_goal --> _move_to_goal_status
    _move_to_goal --> _move_to_goal_result
    _move_to_goal --> _move_to_goal_feedback
    _move_to_goal --> _cmd_vel
    _command_listener --> _cmd_vel
    _cmd_vel --> _move_robot
``` 

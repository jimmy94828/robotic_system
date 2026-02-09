# Object Query

### Communication
- Service

```
# ObjectQuery.srv

# Request
string name                     # object str
---
# Response
bool found                      # object is founded
geometry_msgs/Point position    # location
string message                  # str msg
```

### Usage
```
# server (I build a fake database to query the object location)
ros2 run object_query object_query_server

# client
ros2 run object_query object_query_client <object_name>
```
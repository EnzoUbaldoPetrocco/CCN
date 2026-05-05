# navigation_server.py

## Purpose
Flask REST API server integrating navigation, localization, and conversation management for the Pepper robot.

## Global Configuration
- **Flask App:** Instance with upload/visualization folders
- **ConversationManager:** Global manager instance
- **RRT Parameters:** max_iterations=5000, step_size=6, tolerance=1
- **Robot Config:** start_node at (40, 45), goal_node at (80, 80)
- **Camera Calibration:** Intrinsic matrix, distortion coefficients, marker positions
- **Scaling:** Transforms between world coordinates (meters) and map coordinates (pixels)

## Key Functions

### Coordinate Transformation Functions
- `load_latest_map_metadata()`: Load map scaling and origin from stored JSON
- `transformRadius2WorldCoordinates(value)`: Convert distance from map to world coordinates
- `transformRadius2MapCoordinates(value)`: Convert distance from world to map coordinates
- `transformNode2MapCoordinates(node)`: Convert node position/orientation from world to map
- `transformNode2WorldCoordinates(node)`: Convert node position/orientation from map to world
- `parse_transform_string_to_matrix(transform_str)`: Parse transform string to 4x4 matrix

## REST API Endpoints

### Robot Position Management
- `robot_position()` [POST/GET]: Get/set current robot position
- `start_position()` [POST/GET]: Get/set start position
- `goal_position()` [POST/GET]: Get/set goal position
- `robot_position_using_camera()` [POST]: Estimate robot position from camera image
- `map_image_and_binary_map()` [POST]: Upload map image and get binary version

### Navigation
- `init_navigation_path(TAG, destination_node)`: Initialize pathfinding
- `navigation_path()` [GET]: Get computed path as JSON coordinates
- `navigation_path_with_plot()` [GET]: Get path with matplotlib visualization
- `latest_map_file()` [GET]: Retrieve latest uploaded map
- `nav_go_to()` [POST]: Execute navigation to goal

### Visualization
- `visualize_robot_position_in_map()` [GET]: Plot robot on map
- `visualize_map_position_in_map()` [GET]: Plot map position visualization

### Conversation Control
- `respond()` [POST]: Get robot response to user input
- `phase()` [POST/GET]: Get/set current conversation phase
- `language()` [POST/GET]: Get/set language
- `language_style()` [POST/GET]: Get/set communication style
- `proxemics()` [POST/GET]: Get/set proxemics distance
- `status()` [GET]: Get current system status
- `paradigm()` [POST/GET]: Get/set experiment paradigm
- `is_proxemic_changed()` [POST]: Check if proxemics changed from initial value
- `set_culture()` [POST]: Set culture and update all parameters

### Camera
- `camera()` [POST]: Handle camera image upload

## Technical Details

### Coordinate System Architecture
Implements dual coordinate systems for seamless robot-environment interaction:

**World Coordinates (meters):**
- Real-world metric units
- Used for physical robot movement and proxemics calculations
- Origin: Arbitrary world reference frame

**Map Coordinates (pixels):**
- Image-based discrete coordinates
- Used for path planning on occupancy grids
- Resolution: Configurable pixels per meter

**Transformation Mathematics:**
```
# World to Map
x_map = (x_world - origin_x) / resolution
y_map = (y_world - origin_y) / resolution

# Map to World  
x_world = x_map * resolution + origin_x
y_world = y_map * resolution + origin_y
```

### REST API Specifications

#### Request/Response Formats
- **Content-Type**: `application/json`
- **Authentication**: None (development environment)
- **Error Handling**: HTTP status codes with JSON error messages

#### Example API Calls

**Get Robot Position:**
```http
GET /robot_position
Response: {"x": 1.5, "y": 2.3, "theta": 0.78}
```

**Set Navigation Goal:**
```http
POST /goal_position
Body: {"x": 5.0, "y": 3.2}
Response: {"status": "success"}
```

**Get Navigation Path:**
```http
GET /navigation_path
Response: {"path": [[1.5, 2.3], [2.1, 2.8], [3.2, 3.1], [5.0, 3.2]]}
```

### Path Planning Integration
- **RRT* Algorithm**: Optimal path planning with proxemics constraints
- **Real-time Execution**: Asynchronous path computation
- **Dynamic Replanning**: Path updates based on robot position feedback
- **Human-Aware Navigation**: Incorporates cultural proxemics in cost function

### Conversation Management Integration
- **Multi-threaded**: Separate threads for navigation and conversation
- **State Synchronization**: Coordinated phase transitions between navigation and dialogue
- **Cultural Adaptation**: Dynamic parameter updates affecting both verbal and physical behavior

### Performance Optimization
- **Caching**: Map and calibration data caching
- **Asynchronous Processing**: Non-blocking API responses for long-running operations
- **Resource Management**: Memory-efficient image processing and path storage
- **Scalability**: Modular architecture supporting multiple robot instances

### Security Considerations
- **Input Validation**: Sanitization of all API inputs
- **Rate Limiting**: Protection against excessive API calls
- **Error Handling**: Graceful failure modes with informative error messages
- **Logging**: Comprehensive request/response logging for debugging

### Technical Challenges
- **Real-time Constraints**: Balancing computation time with robot responsiveness
- **Multi-modal Integration**: Coordinating visual, verbal, and physical robot behaviors
- **Network Reliability**: Robust handling of communication failures
- **Cultural Sensitivity**: Ethical implementation of cultural adaptation features</content>
<parameter name="filePath">c:\Users\Utente\Desktop\CCN\Server\navigation_server.md
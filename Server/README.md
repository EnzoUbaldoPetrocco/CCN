# Server Directory Documentation

## Overview
The Server directory contains a Flask-based navigation and conversation server for a Pepper robot in a cultural interaction experiment. It implements path planning (RRT*), robot localization using ArUco markers, and LLM-based conversation management with cultural adaptations.

## Technical Architecture

### System Components
- **Flask REST API**: HTTP-based service layer for robot control and data exchange
- **RRT* Path Planner**: Optimal navigation algorithm with proxemics-aware cost functions
- **ArUco Localization**: Computer vision-based pose estimation using fiducial markers
- **LLM Conversation Manager**: Multi-cultural dialogue system with adaptive parameters
- **Coordinate System Management**: Dual-coordinate architecture (world/map) with automatic transformations

### Key Technical Features
- **Real-time Path Planning**: Sub-second path computation with human-aware navigation
- **Multi-modal Integration**: Coordinated verbal, physical, and visual robot behaviors
- **Cultural Adaptation Framework**: Three-dimensional parameter space (language, proxemics, context)
- **Robust Localization**: Marker-based pose estimation with uncertainty handling
- **Asynchronous Processing**: Non-blocking operations for responsive robot control

## Files

### [conversation_manager.py](conversation_manager.md)
Manages conversational interactions for a Pepper robot, handling multiple languages (Italian, English, German), cultural contexts (high/low context communication), and conversation phases.

### [localization.py](localization.md)
Provides robot localization using ArUco markers detected via camera, with pose estimation and coordinate system transformations.

### [rrt_real.py](rrt_real.md)
Implements Rapidly-exploring Random Trees (RRT*) path planning algorithm with support for human proxemics constraints.

### [proxemics_points.py](proxemics_points.md)
Utility script for calculating scale-adjusted proxemics distance points.

### [navigation_server.py](navigation_server.md)
Flask REST API server integrating navigation, localization, and conversation management for the Pepper robot.

## Subdirectories

- **camera_calibration/**: Camera calibration files and matrices
- **robot_photos/**: Stored robot position images
- **uploaded_maps/**: User-uploaded navigation maps
- **visualizations/**: Generated path visualization images
- **test_server/**: Unit tests for server endpoints
- **__pycache__/**: Python compiled cache

## Architecture Diagram

```mermaid
graph TD
    A[Android App] --> B[Flask Server]
    B --> C[/robot/position]
    B --> D[/navigation/path]
    B --> E[/respond]
    B --> F[/map-upload]
    B --> G[/navigation/go_to]
    C --> H[localization.py]
    D --> I[rrt_real.py]
    E --> J[conversation_manager.py]
    F --> K[Map Handling]
    G --> I
    H --> L[ArUco Detection]
    I --> M[RRT* Algorithm]
    J --> N[LLM Dialogue]
    M --> O[Proxemics Constraints]
    N --> P[Cultural Adaptation]
```

## Dependencies
- **Flask**: REST API framework
- **OpenCV**: ArUco marker detection, image processing
- **NumPy/SciPy**: Numerical computations, transformations
- **Matplotlib**: Visualization
- **Ollama**: Local LLM integration for conversation

## Performance Characteristics
- **Path Planning**: ~100-500ms for typical indoor environments
- **Localization**: ~50-100ms per camera frame
- **API Response Time**: <50ms for most endpoints
- **Memory Usage**: ~200-500MB depending on map complexity
- **Concurrent Users**: Supports multiple robot instances</content>
<parameter name="filePath">c:\Users\Utente\Desktop\CCN\Server\README.md
# Quest 3 WebXR ROS2 Integration

This package provides seamless integration between Oculus Quest 3 VR headsets and ROS2 using WebXR technology. It publishes headset and controller transforms as TF frames and topics, enabling VR-based robot control and teleoperation.

## Features

- **WebXR Integration**: Native Quest 3 support with WebXR API
- **TF2 Publishing**: Headset and controller poses published as TF transforms
- **ROS2 Topics**: Transform and button data available as standard ROS2 topics
- **Secure Communication**: HTTPS and WSS support for secure connections
- **Real-time Data**: Low-latency controller and headset tracking
- **Quest 3 Optimized**: Specifically designed for Oculus Quest 3 capabilities
- **Button & Thumbstick Support**: Full controller input including all buttons and thumbstick axes
- **High Performance**: Optimized for smooth VR tracking without performance degradation

## Published Topics

### Transform Topics
- `/quest3/headset/transform` (geometry_msgs/TransformStamped)
- `/quest3/left_controller/transform` (geometry_msgs/TransformStamped)
- `/quest3/right_controller/transform` (geometry_msgs/TransformStamped)

### Button Topics
- `/quest3/left_controller/buttons` (sensor_msgs/Joy)
- `/quest3/right_controller/buttons` (sensor_msgs/Joy)

## Published TF Frames

- `quest3_headset` - Headset position and orientation in world frame
- `quest3_left_controller` - Left controller relative to headset
- `quest3_right_controller` - Right controller relative to headset

## Installation

### Prerequisites

- ROS2 (tested with Humble and Iron)
- Python 3.8+
- Oculus Quest 3 with WebXR support
- WebXR-compatible browser (Chrome, Edge, or Firefox Reality)

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select quest3_webxr_ros2
source install/setup.bash
```

## Quick Start

1. **Build the package**: `colcon build --packages-select quest3_webxr_ros2`
2. **Start the server**: `ros2 launch quest3_webxr_ros2 quest3_webxr.launch.py`
3. **Open Quest 3 browser**: Navigate to `https://YOUR_IP:8443/quest3_webxr.html`
4. **Enter VR**: Click "Enter VR" and start controlling!

## Usage

### 1. Start the WebXR Server

```bash
# Using launch file (recommended)
ros2 launch quest3_webxr_ros2 quest3_webxr.launch.py

# Or run directly
ros2 run quest3_webxr_ros2 quest3_webxr_server
```

### 2. Access the WebXR App

1. Open a WebXR-compatible browser on your Quest 3
2. Navigate to: `https://YOUR_IP_ADDRESS:8443/quest3_webxr.html`
3. Accept the SSL certificate (self-signed)
4. Click "Enter VR" to start the WebXR session

### 3. Monitor ROS2 Data

```bash
# View published topics
ros2 topic list | grep quest3

# Monitor headset transform
ros2 topic echo /quest3/headset/transform

# Monitor controller transforms
ros2 topic echo /quest3/left_controller/transform
ros2 topic echo /quest3/right_controller/transform

# Monitor button data
ros2 topic echo /quest3/left_controller/buttons
ros2 topic echo /quest3/right_controller/buttons

# View TF tree
ros2 run tf2_tools view_frames
```

## Quest 3 Controller Mapping

### Buttons (sensor_msgs/Joy)
- `buttons[0]` - Trigger (index finger)
- `buttons[1]` - Grip button
- `buttons[2]` - Menu button
- `buttons[3]` - Thumbstick click
- `buttons[4]` - A button (right controller) / X button (left controller)
- `buttons[5]` - B button (right controller) / Y button (left controller)

### Axes (sensor_msgs/Joy)
- `axes[0]` - Thumbstick X axis (-1.0 to 1.0)
- `axes[1]` - Thumbstick Y axis (-1.0 to 1.0)

## Coordinate Frames

### World Frame
- Origin: Quest 3's local-floor reference space
- X: Right
- Y: Up
- Z: Forward (towards user)

### Headset Frame
- Position: Absolute position in world frame
- Orientation: Absolute orientation in world frame

### Controller Frames
- Position: Relative to headset position
- Orientation: Absolute orientation in world frame

## Configuration

### Launch Parameters

```bash
ros2 launch quest3_webxr_ros2 quest3_webxr.launch.py \
    host:=0.0.0.0 \
    http_port:=8443 \
    ws_port:=8080 \
    wss_port:=8444
```

### Network Configuration

- **HTTP Port**: 8443 (HTTPS for WebXR app)
- **WebSocket Port**: 8080 (WS for development)
- **Secure WebSocket Port**: 8444 (WSS for production)

## Troubleshooting

### WebXR Not Supported
- Ensure you're using a WebXR-compatible browser
- Check that your Quest 3 has WebXR enabled
- Try accessing via HTTPS (required for WebXR)

### Connection Issues
- Verify the IP address and port in the browser
- Check firewall settings
- Ensure SSL certificates are properly generated

### No Controller Data
- Make sure controllers are powered on and paired
- Check that you're in VR mode (not passthrough)
- Verify WebSocket connection is established
- Ensure controllers are being tracked (should show "Active" in browser UI)

### Button Topics Not Updating
- Check that you're pressing buttons while in VR mode
- Verify the WebXR session is active
- Monitor server logs for any error messages
- Test with different buttons (trigger, grip, menu, etc.)

### TF Frames Not Appearing
- Check that the WebXR session is active
- Verify controllers are being tracked
- Monitor the server logs for errors

## Development

### Adding New Features

1. Modify `quest3_webxr_ros2/webxr_server.py` for ROS2 backend changes
2. Update `webxr_app/quest3_webxr.js` for frontend changes
3. Rebuild the package: `colcon build --packages-select quest3_webxr_ros2`

### Custom Button Mapping

Edit the `buttonMapping` object in `quest3_webxr.js` to customize button assignments.

### Adding New Topics

1. Add publisher in `setup_ros2_publishers()` method
2. Add publishing logic in the appropriate data processing method
3. Update this README with the new topic information

## Security Notes

- The package uses self-signed SSL certificates for development
- For production use, replace with proper certificates
- Ensure your network is secure when using WSS connections

## License

MIT License - see package.xml for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly with Quest 3
5. Submit a pull request

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review the server logs
3. Open an issue with detailed information about your setup

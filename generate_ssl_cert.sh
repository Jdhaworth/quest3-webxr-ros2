#!/bin/bash

# Generate SSL certificates for Quest 3 WebXR ROS2 Integration
# This script creates self-signed certificates for development use

echo "Generating SSL certificates for Quest 3 WebXR ROS2 Integration..."

# Create webxr_app directory if it doesn't exist
mkdir -p webxr_app

# Generate private key and certificate
openssl req -x509 -newkey rsa:4096 -keyout webxr_app/key.pem -out webxr_app/cert.pem -days 365 -nodes \
    -subj "/C=US/ST=State/L=City/O=Quest3WebXR/CN=localhost" \
    -addext "subjectAltName=DNS:localhost,DNS:*.local,IP:127.0.0.1,IP:0.0.0.0"

if [ $? -eq 0 ]; then
    echo "✅ SSL certificates generated successfully!"
    echo "📁 Certificate: webxr_app/cert.pem"
    echo "🔑 Private key: webxr_app/key.pem"
    echo ""
    echo "⚠️  IMPORTANT SECURITY NOTES:"
    echo "   - These are self-signed certificates for development only"
    echo "   - For production, use proper certificates from a CA"
    echo "   - Never commit these files to version control"
    echo "   - Generate new certificates for each deployment"
    echo ""
    echo "🚀 You can now start the WebXR server:"
    echo "   ros2 launch quest3_webxr_ros2 quest3_webxr.launch.py"
else
    echo "❌ Failed to generate SSL certificates"
    echo "Make sure OpenSSL is installed: sudo apt install openssl"
    exit 1
fi

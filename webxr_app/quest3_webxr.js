// Quest 3 WebXR ROS2 Integration
class Quest3WebXRROS2 {
    constructor() {
        this.xrSession = null;
        this.xrReferenceSpace = null;
        this.gl = null;
        this.canvas = null;
        this.animationId = null;
        this.websocket = null;
        this.controllers = [];
        this.controllerData = {
            headset: { position: null, rotation: null },
            left: { position: null, rotation: null, buttons: {} },
            right: { position: null, rotation: null, buttons: {} }
        };
        
        // Quest 3 specific button mapping
        this.buttonMapping = {
            trigger: 0,    // Index trigger
            grip: 1,       // Grip button
            menu: 2,       // Menu button
            thumbstick: 3, // Thumbstick click
            a: 4,          // A button (right controller)
            b: 5,          // B button (right controller)
            x: 4,          // X button (left controller)
            y: 5           // Y button (left controller)
        };
        
        this.init();
    }
    
    init() {
        this.log('Initializing Quest 3 WebXR ROS2 Integration...');
        
        // Get canvas and setup WebGL context
        this.canvas = document.getElementById('xr-canvas');
        this.setupWebGL();
        
        // Check WebXR support
        this.checkWebXRSupport();
        
        // Set up WebSocket connection
        this.setupWebSocket();
        
        // Set up event listeners
        document.getElementById('enter-vr-btn').addEventListener('click', () => this.enterVR());
        document.getElementById('exit-vr-btn').addEventListener('click', () => this.exitVR());
    }
    
    log(message) {
        const logEl = document.getElementById('log-output');
        const timestamp = new Date().toLocaleTimeString();
        logEl.innerHTML += `<div>[${timestamp}] ${message}</div>`;
        logEl.scrollTop = logEl.scrollHeight;
        console.log('[Quest3 WebXR]', message);
    }
    
    setupWebGL() {
        this.log('Setting up WebGL context...');
        
        try {
            // Create WebGL context with xrCompatible flag and alpha support
            this.gl = this.canvas.getContext('webgl', { 
                xrCompatible: true,
                antialias: true,
                depth: true,
                alpha: true,
                premultipliedAlpha: false
            });
            
            if (!this.gl) {
                this.log('ERROR: WebGL context creation failed');
                document.getElementById('webgl-status').className = 'status error';
                document.getElementById('webgl-status-text').textContent = 'Failed';
                return;
            }
            
            this.log('SUCCESS: WebGL context created with xrCompatible flag');
            document.getElementById('webgl-status').className = 'status success';
            document.getElementById('webgl-status-text').textContent = 'Ready';
            
            // Set canvas size
            this.canvas.width = this.canvas.clientWidth;
            this.canvas.height = this.canvas.clientHeight;
            
            // Basic WebGL setup for transparency
            this.gl.clearColor(0.0, 0.0, 0.0, 0.0); // Transparent background
            this.gl.enable(this.gl.DEPTH_TEST);
            this.gl.enable(this.gl.BLEND);
            this.gl.blendFunc(this.gl.SRC_ALPHA, this.gl.ONE_MINUS_SRC_ALPHA);
            
        } catch (error) {
            this.log('ERROR: WebGL setup failed: ' + error.message);
            document.getElementById('webgl-status').className = 'status error';
            document.getElementById('webgl-status-text').textContent = 'Failed';
        }
    }
    
    checkWebXRSupport() {
        this.log('Checking WebXR support...');
        
        const statusEl = document.getElementById('webxr-status');
        const statusText = document.getElementById('webxr-status-text');
        
        if (!navigator.xr) {
            statusEl.className = 'status error';
            statusText.textContent = 'WebXR not supported';
            this.log('ERROR: WebXR not supported in this browser');
            return;
        }
        
        this.log('WebXR API is available');
        
        // Check for immersive-vr support
        navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
            if (supported) {
                statusEl.className = 'status success';
                statusText.textContent = 'WebXR supported';
                document.getElementById('enter-vr-btn').disabled = false;
                this.log('SUCCESS: WebXR immersive-vr supported');
            } else {
                statusEl.className = 'status error';
                statusText.textContent = 'WebXR not supported';
                this.log('ERROR: WebXR immersive-vr not supported');
            }
        }).catch((error) => {
            statusEl.className = 'status error';
            statusText.textContent = 'WebXR check failed';
            this.log('ERROR: WebXR check failed: ' + error.message);
        });
    }
    
    setupWebSocket() {
        this.log('Setting up WebSocket connection...');
        
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const port = window.location.protocol === 'https:' ? '8444' : '8080';
        const wsUrl = `${protocol}//${window.location.hostname}:${port}`;
        
        this.websocket = new WebSocket(wsUrl);
        
        this.websocket.onopen = () => {
            this.log('SUCCESS: WebSocket connected');
            document.getElementById('ws-status').textContent = 'Connected';
            document.getElementById('connection-status').className = 'status success';
        };
        
        this.websocket.onclose = () => {
            this.log('WebSocket disconnected');
            document.getElementById('ws-status').textContent = 'Disconnected';
            document.getElementById('connection-status').className = 'status error';
        };
        
        this.websocket.onerror = (error) => {
            this.log('ERROR: WebSocket error: ' + error);
            document.getElementById('ws-status').textContent = 'Error';
            document.getElementById('connection-status').className = 'status error';
        };
        
        this.websocket.onmessage = (event) => {
            this.log('Received: ' + event.data);
        };
    }
    
    async enterVR() {
        try {
            this.log('Attempting to enter VR...');
            
            if (!this.gl) {
                throw new Error('WebGL context not available');
            }
            
            // Request XR session with Quest 3 specific features
            this.xrSession = await navigator.xr.requestSession('immersive-vr', {
                requiredFeatures: ['local-floor'],
                optionalFeatures: ['hand-tracking', 'bounded-floor']
            });
            
            this.log('SUCCESS: XR session created');
            
            // Set up XR session
            this.xrSession.addEventListener('end', () => this.onXRSessionEnd());
            
            // Create XR WebGL layer with transparency support for passthrough
            const xrWebGLLayer = new XRWebGLLayer(this.xrSession, this.gl, {
                alpha: true,
                antialias: true,
                depth: true,
                stencil: false,
                ignoreDepthValues: false
            });
            this.xrSession.updateRenderState({ baseLayer: xrWebGLLayer });
            
            this.log('SUCCESS: XR WebGL layer created');
            
            // Request reference space
            this.xrReferenceSpace = await this.xrSession.requestReferenceSpace('local-floor');
            this.log('SUCCESS: Reference space created');
            
            // Update UI
            document.getElementById('enter-vr-btn').disabled = true;
            document.getElementById('exit-vr-btn').disabled = false;
            
            // Update status
            document.getElementById('session-status').className = 'status success';
            document.getElementById('session-status-text').textContent = 'Active';
            
            this.log('SUCCESS: VR session started successfully!');
            
            // Start render loop
            this.startRenderLoop();
            
        } catch (error) {
            this.log('ERROR: Failed to enter VR: ' + error.message);
            this.log('Error details: ' + JSON.stringify(error));
            alert('Failed to enter VR: ' + error.message);
        }
    }
    
    startRenderLoop() {
        const renderLoop = (time, frame) => {
            if (this.xrSession) {
                const session = frame.session;
                const gl = this.gl;
                
                // Get viewer pose
                const pose = frame.getViewerPose(this.xrReferenceSpace);
                
                if (pose) {
                    // Bind the framebuffer
                    gl.bindFramebuffer(gl.FRAMEBUFFER, session.renderState.baseLayer.framebuffer);
                    
                    // Clear the screen
                    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
                    
                    // Render for each view
                    for (const view of pose.views) {
                        const viewport = session.renderState.baseLayer.getViewport(view);
                        gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
                        
                        // Transparent background for passthrough
                        gl.clearColor(0.0, 0.0, 0.0, 0.0);
                        gl.clear(gl.COLOR_BUFFER_BIT);
                    }
                    
                    // Update controller and headset data
                    this.updateQuest3Data(frame);
                }
                
                // Continue the render loop
                session.requestAnimationFrame(renderLoop);
            }
        };
        
        this.xrSession.requestAnimationFrame(renderLoop);
        this.log('Render loop started');
    }
    
    updateQuest3Data(frame) {
        // Get input sources (controllers)
        const inputSources = frame.session.inputSources;
        
        // Get headset pose (viewer pose) for world coordinates
        const viewerPose = frame.getViewerPose(this.xrReferenceSpace);
        if (viewerPose) {
            const headsetTransform = viewerPose.transform;
            this.controllerData.headset = {
                position: {
                    x: headsetTransform.position.x,
                    y: headsetTransform.position.y,
                    z: headsetTransform.position.z
                },
                rotation: {
                    x: headsetTransform.orientation.x,
                    y: headsetTransform.orientation.y,
                    z: headsetTransform.orientation.z,
                    w: headsetTransform.orientation.w
                }
            };
            
            // Update headset status
            document.getElementById('headset-status').textContent = 'Active';
        }
        
        // Process controllers
        for (let i = 0; i < inputSources.length; i++) {
            const inputSource = inputSources[i];
            const hand = inputSource.handedness; // 'left' or 'right'
            
            if (hand === 'left' || hand === 'right') {
                // Update controller status
                document.getElementById(`${hand}-status`).textContent = 'Active';
                
                // Always get button data, regardless of pose availability
                const buttons = this.getControllerButtons(inputSource, hand);
                
                // Get controller pose in world coordinates
                const gripPose = frame.getPose(inputSource.gripSpace, this.xrReferenceSpace);
                if (gripPose && viewerPose) {
                    // Calculate position relative to headset
                    const worldPos = gripPose.transform.position;
                    const headsetPos = viewerPose.transform.position;
                    const headsetRot = viewerPose.transform.orientation;
                    
                    // Calculate relative position (controller - headset)
                    const translatedPos = {
                        x: worldPos.x - headsetPos.x,
                        y: worldPos.y - headsetPos.y,
                        z: worldPos.z - headsetPos.z
                    };
                    
                    // Apply inverse headset rotation to get headset-relative coordinates
                    const relativePos = this.rotateVectorByQuaternion(translatedPos, this.inverseQuaternion(headsetRot));
                    
                    this.controllerData[hand] = {
                        position: relativePos,
                        rotation: {
                            x: gripPose.transform.orientation.x,
                            y: gripPose.transform.orientation.y,
                            z: gripPose.transform.orientation.z,
                            w: gripPose.transform.orientation.w
                        },
                        buttons: buttons
                    };
                } else {
                    // If pose not available, still send button data with default position/rotation
                    this.controllerData[hand] = {
                        position: { x: 0, y: 0, z: 0 },
                        rotation: { x: 0, y: 0, z: 0, w: 1 },
                        buttons: buttons
                    };
                }
                
                // Update UI with controller data
                this.updateControllerUI(hand);
            }
        }
        
        // Send all data to ROS2
        this.sendQuest3Data();
    }
    
    getControllerButtons(inputSource, hand) {
        const buttons = {};
        
        // Debug: Log input source properties (commented out for performance)
        // this.log(`${hand} input source: gamepad=${!!inputSource.gamepad}, targetRayMode=${inputSource.targetRayMode}, handedness=${inputSource.handedness}`);
        
        if (inputSource.gamepad) {
            const gamepad = inputSource.gamepad;
            
            // Quest 3 button mapping
            buttons.trigger = gamepad.buttons[0]?.pressed || false;
            buttons.grip = gamepad.buttons[1]?.pressed || false;
            buttons.menu = gamepad.buttons[2]?.pressed || false;
            buttons.thumbstick = gamepad.buttons[3]?.pressed || false;
            
            // Hand-specific buttons
            if (hand === 'right') {
                buttons.a = gamepad.buttons[4]?.pressed || false;
                buttons.b = gamepad.buttons[5]?.pressed || false;
            } else {
                buttons.x = gamepad.buttons[4]?.pressed || false;
                buttons.y = gamepad.buttons[5]?.pressed || false;
            }
            
            // Thumbstick axes - Quest 3 uses axes[2] and axes[3] for thumbstick
            if (gamepad.axes && gamepad.axes.length >= 4) {
                buttons.thumbstick_x = gamepad.axes[2] || 0.0;
                buttons.thumbstick_y = gamepad.axes[3] || 0.0;
            } else if (gamepad.axes && gamepad.axes.length >= 2) {
                // Fallback to axes[0] and axes[1] if only 2 axes available
                buttons.thumbstick_x = gamepad.axes[0] || 0.0;
                buttons.thumbstick_y = gamepad.axes[1] || 0.0;
            } else {
                buttons.thumbstick_x = 0.0;
                buttons.thumbstick_y = 0.0;
            }
            
            // Debug: Log gamepad info (commented out for performance)
            // if (gamepad.axes) {
            //     this.log(`${hand} controller: ${gamepad.axes.length} axes, values: [${gamepad.axes.join(', ')}]`);
            // } else {
            //     this.log(`${hand} controller: No axes available`);
            // }
            // 
            // // Debug: Log all gamepad properties
            // this.log(`${hand} controller gamepad: buttons=${gamepad.buttons?.length || 0}, axes=${gamepad.axes?.length || 0}`);
        }
        
        return buttons;
    }
    
    updateControllerUI(hand) {
        const pos = this.controllerData[hand].position;
        const buttons = this.controllerData[hand].buttons;
        
        if (pos) {
            const statusText = `Rel Pos: (${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)})`;
            const buttonText = Object.entries(buttons)
                .filter(([key, value]) => value && key !== 'thumbstick_x' && key !== 'thumbstick_y')
                .map(([key, value]) => key)
                .join(', ');
            
            const fullStatus = buttonText ? `${statusText} | Buttons: ${buttonText}` : statusText;
            document.getElementById(`${hand}-status`).textContent = fullStatus;
        }
    }
    
    sendQuest3Data() {
        if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
            const data = {
                type: 'quest3_data',
                timestamp: Date.now(),
                headset: this.controllerData.headset,
                left: this.controllerData.left,
                right: this.controllerData.right
            };
            
            // Debug: Log button data being sent (commented out for performance)
            // if (this.controllerData.left && this.controllerData.left.buttons) {
            //     const leftButtons = Object.entries(this.controllerData.left.buttons)
            //         .filter(([key, value]) => value && key !== 'thumbstick_x' && key !== 'thumbstick_y')
            //         .map(([key, value]) => key);
            //     if (leftButtons.length > 0) {
            //         this.log(`Sending left buttons: ${leftButtons.join(', ')}`);
            //     }
            // }
            // 
            // if (this.controllerData.right && this.controllerData.right.buttons) {
            //     const rightButtons = Object.entries(this.controllerData.right.buttons)
            //         .filter(([key, value]) => value && key !== 'thumbstick_x' && key !== 'thumbstick_y')
            //         .map(([key, value]) => key);
            //     if (rightButtons.length > 0) {
            //         this.log(`Sending right buttons: ${rightButtons.join(', ')}`);
            //     }
            // }
            
            this.websocket.send(JSON.stringify(data));
        }
    }
    
    // Helper function to calculate inverse quaternion
    inverseQuaternion(q) {
        return {
            x: -q.x,
            y: -q.y,
            z: -q.z,
            w: q.w
        };
    }
    
    // Helper function to rotate a vector by a quaternion
    rotateVectorByQuaternion(v, q) {
        // Convert vector to quaternion (w=0)
        const vQuat = { x: v.x, y: v.y, z: v.z, w: 0 };
        
        // q * v * q^-1
        const qInverse = this.inverseQuaternion(q);
        const result = this.multiplyQuaternions(
            this.multiplyQuaternions(q, vQuat),
            qInverse
        );
        
        return {
            x: result.x,
            y: result.y,
            z: result.z
        };
    }
    
    // Helper function to multiply two quaternions
    multiplyQuaternions(q1, q2) {
        return {
            x: q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            y: q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            z: q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
            w: q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        };
    }
    
    exitVR() {
        if (this.xrSession) {
            this.log('Exiting VR...');
            this.xrSession.end();
        }
    }
    
    onXRSessionEnd() {
        this.log('XR session ended');
        this.xrSession = null;
        this.xrReferenceSpace = null;
        
        // Update UI
        document.getElementById('enter-vr-btn').disabled = false;
        document.getElementById('exit-vr-btn').disabled = true;
        
        // Update status
        document.getElementById('session-status').className = 'status';
        document.getElementById('session-status-text').textContent = 'None';
        
        // Reset controller status
        document.getElementById('headset-status').textContent = 'Not detected';
        document.getElementById('left-status').textContent = 'Not detected';
        document.getElementById('right-status').textContent = 'Not detected';
        
        // Clear canvas with transparent background
        if (this.gl) {
            this.gl.clearColor(0.0, 0.0, 0.0, 0.0);
            this.gl.clear(this.gl.COLOR_BUFFER_BIT);
        }
    }
}

// Initialize when page loads
window.addEventListener('load', () => {
    new Quest3WebXRROS2();
});

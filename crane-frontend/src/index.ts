import * as THREE from 'three';
import URDFLoader, { URDFJoint, URDFLink } from 'urdf-loader';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';


// Create the scene
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer();
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

// Set camera position
camera.position.x = 0;
camera.position.y = -15;
camera.position.z = 3.0;

// Controls
const controls = new OrbitControls(camera, renderer.domElement);

// Lighting
scene.background = new THREE.Color(0x008000);
const ambientLight = new THREE.AmbientLight(0xffffff);
scene.add(ambientLight);
// const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
// scene.add(directionalLight);


// Create the geometry for the floor plane
const geometry = new THREE.PlaneGeometry(10, 10);

// Load the texture
const textureLoader = new THREE.TextureLoader();
const floorTexture = textureLoader.load('http://localhost:3000/resources/ground.jpg');

// Create a basic material and set its color
const material = new THREE.MeshBasicMaterial({ map: floorTexture, side: THREE.DoubleSide });

// Create the mesh from the geometry and material
const floor = new THREE.Mesh(geometry, material);

// Rotate the floor to lie flat
// floor.rotation.x = Math.PI / 2;

// Add the floor to the scene
scene.add(floor);

// Create the wall geometry
const wallGeometry = new THREE.PlaneGeometry(10, 5);

// Load the texture
const wallTexture = textureLoader.load('http://localhost:3000/resources/brick.jpeg');

// Create a basic material for the walls
const wallMaterial = new THREE.MeshBasicMaterial({ map: wallTexture, side: THREE.DoubleSide });

// Create and position the walls
const wall1 = new THREE.Mesh(wallGeometry, wallMaterial);
wall1.position.set(5, 0, 2.5);
wall1.rotation.y = Math.PI / 2;
wall1.rotation.z = Math.PI / 2;
scene.add(wall1);

const wall2 = new THREE.Mesh(wallGeometry, wallMaterial);
wall2.position.set(0, 5, 2.5);
wall2.rotation.x = Math.PI / 2;
// wall2.rotation.z = Math.PI / 2;
scene.add(wall2);

const wall3 = new THREE.Mesh(wallGeometry, wallMaterial);
wall3.position.set(-5, 0, 2.5);
wall3.rotation.y = Math.PI / 2;
wall3.rotation.z = Math.PI / 2;
scene.add(wall3);


// Create and position the first axes helper
const worldAxis = new THREE.AxesHelper(5);
worldAxis.position.set(0.0, 0.0, 0.0);
scene.add(worldAxis);

// Create and position the second axes helper
const baseLinkAxis = new THREE.AxesHelper(5);
baseLinkAxis.position.set(0.0, 0.0, 0.0);
scene.add(baseLinkAxis);

// Load the URDF file
const manager = new THREE.LoadingManager();
const urdf_loader = new URDFLoader(manager);
let robot: THREE.Object3D;

urdf_loader.load('http://localhost:3000/resources/awesome_crane1.urdf', (loadedRobot) => {
    robot = loadedRobot;
    scene.add(robot);
    // console.info(robot);
});
console.info("urdf loaded successfulyy")

console.info(scene.getObjectByName("base_link")?.position);

// WebSocket connection
const socket = new WebSocket('ws://localhost:8765');

// Connection opened
socket.addEventListener('open', (event) => {
    console.log('Connected to WebSocket server');
    // Send a command to the server
    socket.send(JSON.stringify({ command: 'start' }));
    });
    
const receivedEEPosition = document.getElementById('eePosition');
const receivedeeCommandStatus = document.getElementById('eeCommandStatus');
const receivedrobotMotionStatus = document.getElementById('robotMotionStatus');
const receivedjointCommandStatus = document.getElementById('jointCommandStatus');
const receivedoriginShift = document.getElementById('originData');
socket.onmessage = (event) => {
    const data = JSON.parse(event.data);
    console.info(data);
    if (data.command === 'update') {
        const joint_positions = data.joint_positions;
        const ee_position = data.ee_position;
        const robot_motion_status = data.robot_motion_status;
        const origin_shift = data.origin_shift
        // Update the UI with the received data
        if (receivedEEPosition){
            receivedEEPosition.textContent = `
            X: ${ee_position[0]},   
            Y: ${ee_position[1]},
            Z: ${ee_position[2]}
            `;
        }
        if (receivedrobotMotionStatus){
            receivedrobotMotionStatus.textContent = `
            status: ${robot_motion_status}`
        }
        if (receivedoriginShift){
            receivedoriginShift.textContent = `
            X: ${origin_shift[0]},
            Y: ${origin_shift[1]},
            Z: ${origin_shift[2]},
            Yaw: ${origin_shift[3]},
            `
        }
        // document.getElementById('originData').textContent = `X: ${origin.ox}, Y: ${origin.oy}, Z: ${origin.oz}`;
        
        // const angles: number[] = msg["joint_positions"];

        // Update crane joint angles based on received data
        var i = 0;
        if (robot) {
            robot.traverse((child) => {
                const link = child as URDFLink;
                if (link.isURDFLink){
                    // console.info(link.name)
                    if (link.name === 'base_link'){
                        // console.info(link.position);
                        const vec = new THREE.Vector3;
                        // console.info(origin_shift);

                        // Set the world coordinates you want to move the parent cube to
                        const targetWorldPosition = new THREE.Vector3(origin_shift[0], origin_shift[1], origin_shift[2]);
                        baseLinkAxis.position.copy(targetWorldPosition)

                        // Move the parent cube to the target world coordinates
                        link.position.copy(targetWorldPosition);

                        // Set the orientation using a quaternion
                        const quaternion = new THREE.Quaternion();
                        quaternion.setFromEuler(new THREE.Euler(0.0, 0.0, origin_shift[3])); // (pitch, yaw, roll)
                        link.quaternion.copy(quaternion);
                        
                    }
                }
                const joint = child as URDFJoint;
                if (joint.isURDFJoint) {
                    // console.info(i);
                    joint.setJointValue(joint_positions[i]);
                    i++;
                }
            });
        }
    }
    if (data.command === 'go_to_response') {
        const go_to_response = data.res;
        if (receivedeeCommandStatus){
            receivedeeCommandStatus.textContent = 
            `Status: ${go_to_response}`;
        }
    }
    if (data.command === 'joint_command_response') {
        const joint_command_response = data.res;
        console.info(joint_command_response)
        if (receivedjointCommandStatus){
            receivedjointCommandStatus.textContent = 
            `Status: ${joint_command_response}`;
        }
    }
};

const sendEECommand = document.getElementById('sendEECommand') as HTMLButtonElement | null;
const sendOriginCommand = document.getElementById('sendOriginCommand') as HTMLButtonElement | null;
const sendJointCommand = document.getElementById('sendJointCommand') as HTMLButtonElement | null;

const xInput = document.getElementById('x') as HTMLInputElement | null;
const yInput = document.getElementById('y') as HTMLInputElement | null;
const zInput = document.getElementById('z') as HTMLInputElement | null;

const oxInput = document.getElementById('ox') as HTMLInputElement | null;
const oyInput = document.getElementById('oy') as HTMLInputElement | null;
const ozInput = document.getElementById('oz') as HTMLInputElement | null;
const oyawInput = document.getElementById('oyaw') as HTMLInputElement | null;

const j1Input = document.getElementById('j1') as HTMLInputElement | null;
const j2Input = document.getElementById('j2') as HTMLInputElement | null;
const j3Input = document.getElementById('j3') as HTMLInputElement | null;
const j4Input = document.getElementById('j4') as HTMLInputElement | null;

if (sendEECommand && xInput && yInput && zInput){
    // Send vector to server
    sendEECommand.addEventListener('click', () => {
    const x = parseFloat(xInput.value);
    const y = parseFloat(yInput.value);
    const z = parseFloat(zInput.value);
    const ee_command = { x, y, z };
    const command = JSON.stringify({ command: 'ee_position', data: { ee_command } });
    socket.send(command);
    console.info(command);
});
}
else {
    console.error('One or more elements are not found in the document.');
}

if (sendOriginCommand && oxInput && oyInput && ozInput && oyawInput){
    // Send  origin to server
    sendOriginCommand.addEventListener('click', () => {
    const ox = parseFloat(oxInput.value);
    const oy = parseFloat(oyInput.value);
    const oz = parseFloat(ozInput.value);
    const oyaw = parseFloat(oyawInput.value);
    const origin = { ox, oy, oz, oyaw };
    const command = JSON.stringify({ command: 'origin_pose', data: { origin } });
    socket.send(command);
    console.info(command);
});
}
else {
    console.error('One or more elements are not found in the document.');
}
if (sendJointCommand && j1Input && j2Input && j3Input && j4Input){
    // Send vector to server
    sendJointCommand.addEventListener('click', () => {
    const j1 = parseFloat(j1Input.value);
    const j2 = parseFloat(j2Input.value);
    const j3 = parseFloat(j3Input.value);
    const j4 = parseFloat(j4Input.value);
    const joint_command = [j1, j2, j3, j4];
    const command = JSON.stringify({ command: 'joint_pose', data: { joint_command } })
    socket.send(command);
    console.info(command);
});
}
else {
    console.error('One or more elements are not found in the document.');
}


// Animation loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();
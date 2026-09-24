import type {Testbed} from "../Testbed";
import {Vector3, Object3D, Mesh, BufferGeometry, BufferAttribute} from "three";
import {GLTFLoader} from "three/examples/jsm/loaders/GLTFLoader";
type RAPIER_API = typeof import("@dimforge/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    testbed.parameters.debugRender = true;

    // Create Ground.
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let groundBody = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(5.0, 0.1, 5.0);
    world.createCollider(colliderDesc, groundBody);

    // Adding the 3d model

    let loader = new GLTFLoader();

    loader.load("./suzanne_blender_monkey.glb", (gltf) => {
        gltf.scene.position.set(0, 1.2, 0);
        gltf.scene.scale.set(3, 3, 3);
        testbed.graphics.scene.add(gltf.scene);
        gltf.scene.updateMatrixWorld(true); // ensure world matrix is up to date
        gltf.scene.traverse((child: Object3D) => {
            if ((child as Mesh).isMesh && (child as Mesh).geometry) {
                const mesh = child as Mesh;
                const geometry = mesh.geometry as BufferGeometry;

                const vertices: number[] = [];
                const indices = new Uint32Array(geometry.index!.array); // assume index is non-null
                const positionAttribute = geometry.getAttribute(
                    "position",
                ) as BufferAttribute;

                mesh.updateWorldMatrix(true, true);

                const v = new Vector3();

                for (let i = 0, l = positionAttribute.count; i < l; i++) {
                    v.fromBufferAttribute(positionAttribute, i);
                    v.applyMatrix4(mesh.matrixWorld);
                    vertices.push(v.x, v.y, v.z);
                }

                const verticesArray = new Float32Array(vertices);

                const rigidBodyDesc = RAPIER.RigidBodyDesc.fixed();
                const rigidBody = world.createRigidBody(rigidBodyDesc);

                const colliderDesc = RAPIER.ColliderDesc.trimesh(
                    verticesArray,
                    indices,
                );
                world.createCollider(colliderDesc, rigidBody);
            }
        });
    });

    testbed.setWorld(world);
    let cameraPosition = {
        eye: {x: 10.0, y: 5.0, z: 10.0},
        target: {x: 0.0, y: 0.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}

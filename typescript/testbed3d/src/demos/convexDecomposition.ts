import type {Testbed} from "../Testbed";
import {
    Vector3,
    Object3D,
    Mesh,
    BufferGeometry,
    BufferAttribute,
    MeshPhongMaterial,
    Color,
} from "three";
import {GLTFLoader} from "three/examples/jsm/loaders/GLTFLoader";
type RAPIER_API = typeof import("@dimforge/rapier3d");

export function initWorld(RAPIER: RAPIER_API, testbed: Testbed) {
    let gravity = new RAPIER.Vector3(0.0, -9.81, 0.0);
    let world = new RAPIER.World(gravity);

    testbed.parameters.debugRender = true;

    // Create Ground.
    let bodyDesc = RAPIER.RigidBodyDesc.fixed();
    let groundBody = world.createRigidBody(bodyDesc);
    let colliderDesc = RAPIER.ColliderDesc.cuboid(10.0, 0.1, 10.0);
    world.createCollider(colliderDesc, groundBody);

    let loader = new GLTFLoader();

    const positions = [
        {x: -4, y: 15, z: 0},
        {x: 0, y: 18, z: 0},
        {x: 4, y: 21, z: 0},
    ];

    const colors = [
        [0xff6b6b, 0x4ecdc4, 0x45b7d1, 0xf9ca24, 0xee5a6f],
        [0x6c5ce7, 0xa29bfe, 0xfd79a8, 0xfdcb6e, 0xe17055],
        [0x00b894, 0x00cec9, 0x0984e3, 0x6c5ce7, 0xfd79a8],
    ];

    positions.forEach((pos, index) => {
        loader.load("./suzanne_blender_monkey.glb", (gltf) => {
            const scale = 2.0;
            gltf.scene.position.set(pos.x, pos.y, pos.z);
            gltf.scene.scale.set(scale, scale, scale);
            gltf.scene.updateMatrixWorld(true);

            const v = new Vector3();
            const meshVertices: number[] = [];
            const meshIndices: number[] = [];

            gltf.scene.traverse((child: Object3D) => {
                if ((child as Mesh).isMesh && (child as Mesh).geometry) {
                    const mesh = child as Mesh;
                    const geometry = mesh.geometry as BufferGeometry;
                    const positionAttribute = geometry.getAttribute(
                        "position",
                    ) as BufferAttribute;
                    const indexArray = geometry.index?.array;

                    if (!indexArray) return;

                    const baseIndex = meshVertices.length / 3;

                    for (let i = 0, l = positionAttribute.count; i < l; i++) {
                        v.fromBufferAttribute(positionAttribute, i);
                        v.applyMatrix4(mesh.matrixWorld);
                        meshVertices.push(v.x, v.y, v.z);
                    }

                    for (let i = 0; i < indexArray.length; i++) {
                        meshIndices.push(indexArray[i] + baseIndex);
                    }
                }
            });

            const colliderDesc = RAPIER.ColliderDesc.convexDecomposition(
                new Float32Array(meshVertices),
                new Uint32Array(meshIndices),
            );

            if (colliderDesc) {
                const rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic();
                const rigidBody = world.createRigidBody(rigidBodyDesc);
                world.createCollider(colliderDesc, rigidBody);
                rigidBody.setAngvel(
                    new RAPIER.Vector3(
                        (Math.random() - 0.5) * 2,
                        (Math.random() - 0.5) * 2,
                        (Math.random() - 0.5) * 2,
                    ),
                    true,
                );

                gltf.scene.traverse((child: Object3D) => {
                    if ((child as Mesh).isMesh) {
                        const mesh = child as Mesh;
                        const geometry = mesh.geometry as BufferGeometry;
                        const vertexColors = new Float32Array(
                            geometry.attributes.position.count * 3,
                        );
                        const colorPalette = colors[index];

                        for (
                            let i = 0;
                            i < geometry.attributes.position.count;
                            i++
                        ) {
                            const color = new Color(
                                colorPalette[
                                    Math.floor(
                                        (i /
                                            geometry.attributes.position
                                                .count) *
                                            colorPalette.length,
                                    )
                                ],
                            );
                            vertexColors[i * 3] = color.r;
                            vertexColors[i * 3 + 1] = color.g;
                            vertexColors[i * 3 + 2] = color.b;
                        }

                        geometry.setAttribute(
                            "color",
                            new BufferAttribute(vertexColors, 3),
                        );
                        mesh.material = new MeshPhongMaterial({
                            vertexColors: true,
                            flatShading: false,
                        });
                    }
                });

                testbed.graphics.scene.add(gltf.scene);
            }
        });
    });

    for (let i = 0; i < 10; i++) {
        const x = (Math.random() - 0.5) * 8;
        const y = 3 + Math.random() * 5;
        const z = (Math.random() - 0.5) * 8;
        const size = 0.3 + Math.random() * 0.4;

        const bodyDesc = RAPIER.RigidBodyDesc.dynamic().setTranslation(x, y, z);
        const body = world.createRigidBody(bodyDesc);
        const colliderDesc = RAPIER.ColliderDesc.cuboid(size, size, size);
        world.createCollider(colliderDesc, body);
    }

    testbed.setWorld(world);

    let cameraPosition = {
        eye: {x: 15.0, y: 10.0, z: 15.0},
        target: {x: 0.0, y: 5.0, z: 0.0},
    };
    testbed.lookAt(cameraPosition);
}

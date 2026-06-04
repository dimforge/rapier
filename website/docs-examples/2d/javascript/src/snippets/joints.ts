import RAPIER from '@dimforge/rapier2d';


let world = new RAPIER.World({ x: 0.0, y: -9.81 });
let rigidBodyDesc = RAPIER.RigidBodyDesc.dynamic();
let body1 = world.createRigidBody(rigidBodyDesc);
let body2 = world.createRigidBody(rigidBodyDesc);

{

    // DOCUSAURUS: FixedJoint start
    let params = RAPIER.JointData.fixed({ x: 0.0, y: 0.0 }, 0.0, { x: 0.0, y: -2.0 }, 0.0);
    let joint = world.createImpulseJoint(params, body1, body2, true);
    // DOCUSAURUS: FixedJoint stop
}

{
    // DOCUSAURUS: RevoluteJoint start
    let params = RAPIER.JointData.revolute({ x: 0.0, y: 1.0 }, { x: 0.0, y: -3.0 });
    let joint = world.createImpulseJoint(params, body1, body2, true);
    // DOCUSAURUS: RevoluteJoint stop
}

{
    // DOCUSAURUS: PrismaticJoint start
    let x = { x: 1.0, y: 0.0 };
    let params = RAPIER.JointData.prismatic({ x: 0.0, y: 0.0 }, x, { x: 0.0, y: -3.0 });
    params.limitsEnabled = true;
    params.limits = [-2.0, 5.0];
    let joint = world.createImpulseJoint(params, body1, body2, true);
    // DOCUSAURUS: PrismaticJoint stop
}

{
    // DOCUSAURUS: Motor start
    let x = { x: 1.0, y: 0.0 };
    let params = RAPIER.JointData.prismatic({ x: 0.0, y: 0.0 }, { x: 0.0, y: -3.0 }, x);
    let joint = world.createImpulseJoint(params, body1, body2, true);
    (joint as RAPIER.PrismaticImpulseJoint).configureMotorVelocity(1.0, 0.5);
    // DOCUSAURUS: Motor stop
}



let template = {
  '<Templates>': [
    'user_guides/templates_injected/getting_started',
    'user_guides/templates_injected/getting_started_bevy',
    'user_guides/templates_injected/getting_started_js',
    'user_guides/templates_injected/introduction_to_nalgebra',
    'user_guides/templates_injected/simulation_structures',
    'user_guides/templates_injected/rigid_bodies',
    'user_guides/templates_injected/colliders',
    'user_guides/templates_injected/joints',
    'user_guides/templates_injected/joint_constraints',
    'user_guides/templates_injected/character_controller',
    'user_guides/templates_injected/scene_queries',
    'user_guides/templates_injected/advanced_collision_detection',
    'user_guides/templates_injected/advanced_collision_detection_js',
    // 'user_guides/templates_injected/integration_parameters',
    'user_guides/templates_injected/serialization',
    'user_guides/templates_injected/determinism',
    'user_guides/templates_injected/common_mistakes',
    // 'user_guides/templates_injected/the_rapier_testbed',
    // 'user_guides/templates_injected/common_recipes',
  ]
};

let specialized_guides = {
  'Rust (0.29)': [
    'user_guides/rust/getting_started',
    'user_guides/rust/introduction_to_nalgebra',
    'user_guides/rust/simulation_structures',
    'user_guides/rust/rigid_bodies',
    'user_guides/rust/colliders',
    'user_guides/rust/joints',
    'user_guides/rust/joint_constraints',
    'user_guides/rust/character_controller',
    'user_guides/rust/scene_queries',
    'user_guides/rust/advanced_collision_detection',
    // 'user_guides/rust/integration_parameters',
    'user_guides/rust/serialization',
    'user_guides/rust/determinism',
    'user_guides/rust/common_mistakes',
    // 'user_guides/rust/the_rapier_testbed',
    // 'user_guides/rust/common_recipes',
  ],
  'Bevy Plugin (0.30)': [
    'user_guides/bevy_plugin/getting_started_bevy',
    // 'user_guides/bevy_plugin/simulation_structures',
    'user_guides/bevy_plugin/rigid_bodies',
    'user_guides/bevy_plugin/colliders',
    'user_guides/bevy_plugin/joints',
    'user_guides/bevy_plugin/joint_constraints',
    'user_guides/bevy_plugin/character_controller',
    'user_guides/bevy_plugin/scene_queries',
    'user_guides/bevy_plugin/advanced_collision_detection',

    // bevy specific
    'user_guides/bevy_plugin/multiple_contexts',
    'user_guides/bevy_plugin/common_mistakes',
  ],
  'JavaScript (0.17)': [
    'user_guides/javascript/getting_started_js',
    'user_guides/javascript/rigid_bodies',
    'user_guides/javascript/colliders',
    'user_guides/javascript/joints',
    'user_guides/javascript/joint_constraints',
    'user_guides/javascript/character_controller',
    'user_guides/javascript/scene_queries',
    'user_guides/javascript/advanced_collision_detection_js',
    'user_guides/javascript/serialization',
    'user_guides/javascript/determinism',
    'user_guides/javascript/common_mistakes',
  ],
};

let user_guides;

if (!process.env.PUBLISH_MODE) {
  user_guides = template;
} else {
  user_guides = specialized_guides;
}


const config = {
  docs: [
    'about_rapier',
    {
      'User Guides': [user_guides],
    },
    {
      'API Documentation': [
        'api/javascript/JavaScript2D',
        'api/javascript/JavaScript3D',
        {
          type: 'link',
          label: 'bevy_rapier2d',
          href: 'https://docs.rs/bevy_rapier2d'
        },
        {
          type: 'link',
          label: 'bevy_rapier3d',
          href: 'https://docs.rs/bevy_rapier3d'
        },
        {
          type: 'link',
          label: 'rapier2d',
          href: 'https://docs.rs/rapier2d'
        },
        {
          type: 'link',
          label: 'rapier3d',
          href: 'https://docs.rs/rapier3d'
        },
        {
          type: 'link',
          label: 'rapier2d-f64',
          href: 'https://docs.rs/rapier2d-f64'
        },
        {
          type: 'link',
          label: 'rapier3d-f64',
          href: 'https://docs.rs/rapier3d-f64'
        },
      ],
    }
  ],
};

export default config;
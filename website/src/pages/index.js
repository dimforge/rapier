import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

const features = [
  {
    title: <>Multi-platforms physics simulations</>,
    imageUrl: 'img/feature_contact_models.svg',
    description: (
      <>
        Fast physics for games, animation and robotics. Runs cross-platform,
        including the official support of web platforms.
      </>
    ),
  },
  {
    title: <>Joint constraints</>,
    imageUrl: 'img/feature_multibody_and_constraints.svg',
    description: (
      <>
        Constraint the relative motion of two rigid-bodies using
        force-based joints. Simulate ragdolls, robots, vehicles,
        etc.
      </>
    ),
  },
  {
    title: <>Contact events and scene queries</>,
    imageUrl: 'img/feature_event_handling_and_sensors.svg',
    description: (
      <>
        Handle collision and intersection events or use scene queries to perform
        ray-casting, sweep-tests, intersection tests, etc.
      </>
    ),
  },
  // {
  //     title: <>Continuous collision detection</>,
  //     imageUrl: 'img/feature_continuous_collision_detection.svg',
  //     description: (
  //         <>
  //             Use collision detection (CCD) on any rigid-body to avoid missing contacts.
  //             CCD is also supported on sensors.
  //         </>
  //     ),
  // },
  {
    title: <>SIMD and parallelism</>,
    imageUrl: 'img/feature_performances_tuning.svg',
    description: (
      <>
        Optionally enable SIMD optimizations and/or parallelism to get the best
        performance your CPU can offer to simulate complex scenes.
      </>
    ),
  },
  {
    title: <>WebAssembly and JavaScript</>,
    imageUrl: 'img/feature_wasm.svg',
    description: (
      <>
        Use our official NPM packages published behind
        the&nbsp;<a href="https://www.npmjs.com/search?q=%40dimforge">@dimforge</a>&nbsp;namespace and run Rapier
        in any modern browser.
      </>
    ),
  },
  {
    title: <>Cross-platform determinism</>,
    imageUrl: 'img/undraw_determinism.svg',
    description: (
      <>
        Optionally make Rapier cross-platform deterministic on all
        IEEE 754-2008 compliant 32- and 64-bits platforms.
      </>
    ),
  },
  {
    title: <>Forever free and Open-Source</>,
    imageUrl: 'img/undraw_open_source.svg',
    description: (
      <>
        Built with a FOSS mindset, we aim to empower the Rust and web communities
        with an efficient physics simulation framework.
      </>
    ),
  },
];

function Feature({ imageUrl, title, description }) {
  const imgUrl = useBaseUrl(imageUrl);
  return (
    <div className={clsx('col col--4', styles.feature)}>
      {imgUrl && (
        <div className="text--center">
          <img className={styles.featureImage} src={imgUrl} alt={title} />
        </div>
      )}
      <h3>{title}</h3>
      <p>{description}</p>
    </div>
  );
}

function Home() {
  const context = useDocusaurusContext();
  const { siteConfig = {} } = context;
  return (
    <Layout
      title={`${siteConfig.title} physics engine`}
      description="Fast and cross-platform physics engine">
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <div className="">
            <img src="img/rapier_logo_color_textpath.svg" width="70%" alt="Project Logo" />
          </div>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className={clsx(
                'button button--outline button--lg --ifm-color-prim force-border', /*button--secondary*/
                styles.getStarted,
              )}
              to={useBaseUrl('docs/')}>
              Get Started
            </Link>
          </div>
        </div>
      </header>
      <main>
        {features && features.length > 0 && (
          <section className={styles.features}>
            <div className="container">
              <div className="row">
                {features.map((props, idx) => (
                  <Feature key={idx} {...props} />
                ))}
              </div>
            </div>
          </section>
        )}
      </main>
    </Layout>
  );
}

export default Home;

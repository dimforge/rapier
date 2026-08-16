import React, {useEffect, useState, useRef} from 'react';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './demos.module.css';

// Each demo is a full all_examples app, compiled to WASM, with its own
// built-in example picker UI.
const demos = [
  {
    name: '3d',
    demo: 'all_examples3',
    title: '3D Demos',
    description: 'Rigid-body dynamics, joints and character control in 3D',
    source: 'https://github.com/dimforge/rapier/tree/master/examples3d',
  },
  {
    name: '2d',
    demo: 'all_examples2',
    title: '2D Demos',
    description: 'Rigid-body dynamics, joints and character control in 2D',
    source: 'https://github.com/dimforge/rapier/tree/master/examples2d',
  },
];

// What prevents the demos from running here, if anything. Resolved on the
// client only (`navigator` doesn't exist while the site is pre-rendered), so
// `undefined` means "not determined yet".
function detectBlocker() {
  // The testbed renders through wgpu, which targets WebGPU on the web.
  if (!navigator.gpu) return 'webgpu';
  return null;
}

export default function Demos() {
  const [selected, setSelected] = useState(null);
  const [activeDemo, setActiveDemo] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [blocker, setBlocker] = useState(undefined);
  const iframeRef = useRef(null);
  const demosBaseUrl = useBaseUrl('/demos/');

  useEffect(() => {
    setBlocker(detectBlocker());
  }, []);

  // Handle URL hash for deep linking.
  useEffect(() => {
    const hash = window.location.hash.slice(1);
    if (hash && demos.some((d) => d.name === hash)) {
      setSelected(hash);
    } else {
      setSelected('3d');
    }

    const handleHashChange = () => {
      const newHash = window.location.hash.slice(1);
      if (newHash && demos.some((d) => d.name === newHash)) setSelected(newHash);
    };

    window.addEventListener('hashchange', handleHashChange);
    return () => window.removeEventListener('hashchange', handleHashChange);
  }, []);

  // Handle demo transitions: clear the iframe first to release the GPU context.
  useEffect(() => {
    // Nothing is loaded until the browser is known to support the demos
    // (`undefined` = still unknown, non-null = unsupported).
    if (blocker !== null) return;
    if (selected === activeDemo) return;

    setIsLoading(true);

    if (iframeRef.current) {
      iframeRef.current.src = 'about:blank';
    }
    setActiveDemo(null);

    // Wait for the iframe to be cleared and the GPU context to be released.
    const timer = setTimeout(() => {
      setActiveDemo(selected);
      setIsLoading(false);
    }, 500);

    return () => clearTimeout(timer);
  }, [selected, blocker]);

  const handleSelect = (name) => {
    setSelected(name);
    window.location.hash = name;
  };

  const current = demos.find((d) => d.name === selected);

  return (
    <Layout
      title="Demos"
      description="Interactive Rapier physics demos running in your browser"
      noFooter>
      <div className={styles.container}>
        <div className={styles.toolbar}>
          <div className={styles.tabs}>
            {demos.map((demo) => (
              <button
                key={demo.name}
                className={`${styles.tab} ${selected === demo.name ? styles.tabSelected : ''}`}
                onClick={() => handleSelect(demo.name)}>
                {demo.title}
              </button>
            ))}
          </div>
          {!blocker && (
            <span className={styles.hint}>
              Pick individual examples from the panel inside the viewer. First
              load may take a while (the whole example suite ships as a single
              large WASM module).
            </span>
          )}
        </div>

        <div className={styles.viewer}>
          {blocker === 'webgpu' ? (
            <div className={styles.unsupported}>
              <h2>WebGPU is required</h2>
              <p>
                These demos render through WebGPU, which is not available in
                this browser. See{' '}
                <a
                  href="https://caniuse.com/webgpu"
                  target="_blank"
                  rel="noopener noreferrer">
                  caniuse.com/webgpu
                </a>{' '}
                for browser support.
              </p>
              <p>
                On Firefox, enable <code>dom.webgpu.enabled</code> in{' '}
                <code>about:config</code>.
              </p>
            </div>
          ) : activeDemo ? (
            <>
              <iframe
                ref={iframeRef}
                key={activeDemo}
                src={`${demosBaseUrl}${demos.find((d) => d.name === activeDemo)?.demo}/`}
                title={activeDemo}
                className={styles.viewerFrame}
              />
              <div className={styles.viewerControls}>
                <a
                  href={current?.source}
                  target="_blank"
                  rel="noopener noreferrer"
                  className={styles.sourceLink}>
                  &lt;/&gt; Source
                </a>
              </div>
            </>
          ) : isLoading ? (
            <div className={styles.placeholder}>Loading...</div>
          ) : (
            <div className={styles.placeholder}>Select a demo</div>
          )}
        </div>
      </div>
    </Layout>
  );
}

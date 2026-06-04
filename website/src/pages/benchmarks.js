import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function iframeLoaded() {
    if (typeof document !== 'undefined') {
        var iFrameID = document.getElementById('bench-iframe');
        if (iFrameID) {
            let newHeight = (iFrameID.contentWindow.document.body.scrollHeight + 100) + "px";
            if (iFrameID.height != newHeight)
                iFrameID.height = newHeight;
        }
    }
}

function Benchmarks() {
  const context = useDocusaurusContext();
  const {siteConfig = {}} = context;
  const benchUrl = "/benchmarks3d/index.html";
  setInterval(function(){ iframeLoaded(); }, 3000);

  return (
    <Layout
      title={`${siteConfig.title} physics engine`}
      description="Fast and cross-platform physics engine">
      <main>
        <iframe id='bench-iframe' onLoad={iframeLoaded} src={benchUrl}></iframe>
      </main>
    </Layout>
  );
}

export default Benchmarks;

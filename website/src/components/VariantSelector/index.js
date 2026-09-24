import React, {useEffect} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {useLocation} from '@docusaurus/router';
import {useDoc, useDocsVersion} from '@docusaurus/plugin-content-docs/client';
import {variants, docId, parseDocId, correspondingDoc} from '@site/src/userGuideVariants';
import styles from './styles.module.css';

// The user-guide page of the current doc, or `undefined` if it isn't a user-guide page.
export function useUserGuidePage() {
  const {metadata} = useDoc();
  return parseDocId(metadata.id);
}

// Distance from the top of the window to the line considered as being read: right under the variant selector when
// the navbar is hidden (the navbar hides and shows itself while scrolling, which must not move this line).
function readingLine() {
  const bar = document.querySelector(`.${styles.variantSelector}`);
  return bar ? bar.offsetHeight : 0;
}

// The headings of the page (except the hidden ones, e.g. in an inactive tab), with their position in the document.
function documentHeadings() {
  return [...document.querySelectorAll('.theme-doc-markdown :is(h1, h2, h3, h4, h5, h6)[id]')]
    .filter((heading) => heading.getClientRects().length > 0)
    .map((heading) => ({id: heading.id, top: heading.getBoundingClientRect().top + window.scrollY}))
    .sort((a, b) => a.top - b.top);
}

// Where the page is read: in the section at the reading line, as the fraction of that section
// scrolled past. Switching to another variant scrolls its page to the same fraction of the same section, since the
// same section can be longer or shorter in another variant. The headings above are kept to fall back on the nearest one
// the other variant has.
function readingPosition() {
  const headings = documentHeadings();
  const reading = readingLine() + window.scrollY;
  const next = headings.findIndex((heading) => heading.top > reading + 1);
  const previous = (next === -1 ? headings.length : next) - 1;
  if (previous < 0) {
    return {scrollY: window.scrollY, sections: []};
  }
  const end = next === -1 ? document.documentElement.scrollHeight : headings[next].top;
  const fraction = Math.min(1, (reading - headings[previous].top) / Math.max(1, end - headings[previous].top));
  const sections = headings.slice(0, previous + 1).reverse().map((heading) => heading.id);
  return {scrollY: window.scrollY, sections, fraction};
}

// The scroll position to restore on the page of the next variant, set when a variant is selected.
let pendingScroll = null;

// The scroll offset of the page of the new variant showing the same reading position.
function restoredScrollY(position) {
  const headings = documentHeadings();
  for (const [index, id] of position.sections.entries()) {
    const section = headings.findIndex((heading) => heading.id === id);
    if (section !== -1) {
      const start = headings[section].top;
      const end = section + 1 < headings.length ? headings[section + 1].top : document.documentElement.scrollHeight;
      // Only the section being read keeps its reading fraction: a fallback section is read from its start.
      const reading = start + (index === 0 ? position.fraction * (end - start) : 0);
      return reading - readingLine();
    }
  }
  return position.sections.length === 0 ? position.scrollY : 0;
}

// Scrolls to the reading position, and again whenever the page is resized while it settles (images, code
// highlighting), until the reader scrolls. The browser's own scroll adjustments on resizes (scroll anchoring) must not
// be mistaken for the reader's: only the reader's inputs stop it.
function restoreScroll(position) {
  const content = document.querySelector('.theme-doc-markdown');
  const inputs = ['wheel', 'touchstart', 'keydown', 'mousedown'];
  const apply = () => window.scrollTo(0, restoredScrollY(position));
  const observer = typeof ResizeObserver !== 'undefined' && content ? new ResizeObserver(apply) : null;
  const stop = () => {
    observer?.disconnect();
    inputs.forEach((input) => window.removeEventListener(input, stop, true));
  };
  apply();
  if (observer) {
    observer.observe(content);
    inputs.forEach((input) => window.addEventListener(input, stop, true));
    setTimeout(stop, 3000);
  }
}

// Bar on top of the user-guide pages, switching the page shown to another variant of the guide.
export default function VariantSelector() {
  const current = useUserGuidePage();
  const {docs} = useDocsVersion();
  const {hash, pathname} = useLocation();

  useEffect(() => {
    if (pendingScroll && pendingScroll.pathname === pathname) {
      const scroll = pendingScroll;
      pendingScroll = null;
      // Let the router finish its own scrolling (to the top, or to the hash) first.
      requestAnimationFrame(() => requestAnimationFrame(() => restoreScroll(scroll.position)));
    }
  }, [pathname]);

  if (!current) {
    return null;
  }

  return (
    <nav className={styles.variantSelector} aria-label="User-guide variant">
      <div className={styles.variants}>
        {variants.map((variant) => {
          // Pages outside the sidebar of their variant (but linked from it) have no correspondence: try the same name.
          const name = [correspondingDoc(current.variant.id, current.name, variant.id), current.name].find(
            (candidate) => candidate !== undefined && docs[docId(variant, candidate)] !== undefined,
          );
          const label = variant.version ? `${variant.label} ${variant.version}` : variant.label;
          if (name === undefined) {
            return (
              <span
                key={variant.id}
                className={clsx(styles.variant, styles.missing)}
                aria-disabled="true"
                title={`${label}: this page isn't part of this guide`}>
                {variant.label}
              </span>
            );
          }
          const to = `/docs/${docId(variant, name)}${hash}`;
          const selected = variant.id === current.variant.id;
          return (
            <Link
              key={variant.id}
              to={to}
              className={clsx(styles.variant, selected && styles.selected)}
              aria-current={selected ? 'page' : undefined}
              onClick={(event) => {
                if (event.button === 0 && !event.metaKey && !event.ctrlKey && !event.shiftKey) {
                  pendingScroll = {pathname: to.split('#')[0], position: readingPosition()};
                }
              }}
              title={label}>
              {variant.label}
            </Link>
          );
        })}
      </div>
    </nav>
  );
}

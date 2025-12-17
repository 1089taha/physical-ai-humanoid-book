import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">
          An AI-native textbook exploring how intelligence moves from software into the physical world
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading the Book
          </Link>
        </div>
      </div>
    </header>
  );
}

function ValuePropositionSection() {
  const features = [
    {
      title: 'Physical AI & Embodiment',
      description: 'Understanding how artificial intelligence manifests through physical form and interaction with the real world.',
    },
    {
      title: 'Simulation → Real-World Robotics',
      description: 'Bridging the gap between virtual environments and tangible robotic systems for robust real-world deployment.',
    },
    {
      title: 'Vision-Language-Action Systems',
      description: 'Exploring integrated systems that perceive, reason, and act in complex physical environments.',
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <div key={idx} className={clsx('col col--4')}>
              <div className="text--center padding-horiz--md">
                <h3>{feature.title}</h3>
                <p>{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): React.ReactElement {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`AI-Native Textbook`}
      description="An AI-native textbook exploring Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <ValuePropositionSection />
      </main>
    </Layout>
  );
}
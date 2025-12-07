import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI Focus',
    description: (
      <>
        Learn how to integrate artificial intelligence directly with physical systems,
        creating embodied AI agents that interact with the real world.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    description: (
      <>
        Explore the design, control, and intelligence systems needed for human-like
        robots that can operate in human environments.
      </>
    ),
  },
  {
    title: 'Agent-Native Approach',
    description: (
      <>
        Discover how to build systems using autonomous agents that work together
        to achieve complex robotic behaviors.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
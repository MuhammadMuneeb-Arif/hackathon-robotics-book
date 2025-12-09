import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'The Robotic Nervous System (ROS 2)',
    Svg: require('@site/static/img/module1.svg').default,
    description: (
      <>
        Learn the foundations of robot communication using ROS 2. Understand nodes, topics, services, and real-time robot control.
      </>
    ),
  },
  {
    title: 'Simulation Environments (Gazebo & Isaac Sim)',
    Svg: require('@site/static/img/module2.svg').default,
    description: (
      <>
        Build and test full robot simulations with physics, sensors, and environment interaction using Gazebo and NVIDIA Isaac.
      </>
    ),
  },
  {
    title: 'Humanoid Control & Locomotion',
    Svg: require('@site/static/img/module3.svg').default,
    description: (
      <>
        Explore balance, walking, motion planning, inverse kinematics, and dynamic controllers for humanoid robots.
      </>
    ),
  },
  {
    title: 'Physical AI & Human–Robot Interaction',
    Svg: require('@site/static/img/module4.svg').default,
    description: (
      <>
        Discover how robots perceive, reason, and respond in the physical world using AI, vision systems, and natural interaction models.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );  
}

export default function HomepageFeatures(): ReactNode {
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

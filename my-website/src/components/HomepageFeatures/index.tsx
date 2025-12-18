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
    title: 'ROS 2: The Robot Operating System',
    Svg: require('@site/static/img/module1.svg').default,
    description: (
      <>
        Master the backbone of modern robotics with ROS 2. Learn about nodes, topics, services, and actions for seamless robot communication and real-time control systems.
      </>
    ),
  },
  {
    title: 'Digital Twins: Gazebo & Isaac Sim',
    Svg: require('@site/static/img/module2.svg').default,
    description: (
      <>
        Create realistic robot simulations with physics engines, sensor models, and complex environments. Test your robots virtually before deploying in the real world.
      </>
    ),
  },
  {
    title: 'Humanoid Motion & Control',
    Svg: require('@site/static/img/module3.svg').default,
    description: (
      <>
        Engineer dynamic balance, walking gaits, and locomotion patterns. Master inverse kinematics and motion planning for sophisticated humanoid robot movement.
      </>
    ),
  },
  {
    title: 'AI Perception & Interaction',
    Svg: require('@site/static/img/module4.svg').default,
    description: (
      <>
        Integrate computer vision, sensor fusion, and machine learning for robots that perceive, understand, and interact intelligently with their environment.
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

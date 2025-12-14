import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Simulation-First Learning',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn robotics through safe, repeatable simulation environments before real-world deployment.
        Master Gazebo, Unity, and NVIDIA Isaac Sim for embodied intelligence.
      </>
    ),
  },
  {
    title: 'Industry-Relevant Technologies',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Focus exclusively on tools and frameworks used in professional robotics development
        including ROS 2, Isaac ROS, Vision-Language-Action models, and more.
      </>
    ),
  },
  {
    title: 'Hands-On Mastery',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Every concept includes runnable code examples, system diagrams, and practical exercises
        that students can execute on standard hardware.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
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

import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './AIEnhancedHero.module.css';

const AIEnhancedFeature = ({ imageUrl, title, description }) => {
  const imgUrl = useBaseUrl(imageUrl);
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className={styles.featureImageContainer}>
        {imgUrl && (
          <img className={styles.featureImage} src={imgUrl} alt={title} />
        )}
      </div>
      <h3 className={styles.featureTitle}>{title}</h3>
      <p className={styles.featureDescription}>{description}</p>
    </div>
  );
};

const AIEnhancedHero = () => {
  const { siteConfig } = useDocusaurusContext();

  const features = [
    {
      title: '🤖 Embodied Intelligence',
      description: 'Learn how AI emerges from the interaction between digital brains and physical bodies, creating truly intelligent robotic systems.',
      imageUrl: 'img/embodied-intelligence.svg',
    },
    {
      title: '🌐 ROS 2 Ecosystem',
      description: 'Master the Robot Operating System 2 for building distributed robotic applications with professional-grade communication.',
      imageUrl: 'img/ros2-ecosystem.svg',
    },
    {
      title: '🧠 AI-Robot Integration',
      description: 'Discover how to integrate advanced AI models with robotic platforms for autonomous decision-making and control.',
      imageUrl: 'img/ai-robot-integration.svg',
    },
  ];

  return (
    <>
      <header className={clsx('hero hero--primary', styles.heroBanner)}>
        <div className="container">
          <div className={styles.heroContent}>
            <h1 className={clsx('hero__title', styles.heroTitle)}>
              {siteConfig.title}
            </h1>
            <p className={clsx('hero__subtitle', styles.heroSubtitle)}>
              {siteConfig.tagline}
            </p>
            <div className={styles.buttons}>
              <Link
                className={clsx(
                  'button button--secondary button--lg',
                  styles.getStartedButton,
                )}
                to="/docs/">
                Begin Learning Journey
              </Link>
              <Link
                className={clsx(
                  'button button--outline button--secondary button--lg',
                  styles.exploreButton,
                )}
                to="/docs/module-1/">
                Explore Modules
              </Link>
            </div>
          </div>

          <div className={styles.aiAnimation}>
            <div className={styles.neuralNetwork}></div>
            <div className={styles.robotIcon}>🤖</div>
            <div className={styles.brainIcon}>🧠</div>
            <div className={styles.connectionLine}></div>
          </div>
        </div>

        <div className={styles.heroBackgroundPattern}>
          <div className={styles.hexGrid}></div>
        </div>
      </header>

      <section className={styles.featuresSection}>
        <div className="container">
          <div className="row">
            {features.map((props, idx) => (
              <AIEnhancedFeature key={idx} {...props} />
            ))}
          </div>
        </div>
      </section>
    </>
  );
};

export default AIEnhancedHero;
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './AIPoweredCard.module.css';

const AIPoweredCard = ({
  title,
  children,
  variant = 'default',
  showAIInsights = true,
  aiInsight = 'This section contains important concepts that are foundational to understanding embodied AI systems.'
}) => {
  const [showInsight, setShowInsight] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  const handleAIInsightClick = () => {
    setIsLoading(true);
    // Simulate AI processing delay
    setTimeout(() => {
      setIsLoading(false);
      setShowInsight(!showInsight);
    }, 500);
  };

  return (
    <div className={clsx(styles.aiPoweredCard, styles[`variant-${variant}`])}>
      <div className={styles.cardHeader}>
        {title && <h3 className={styles.cardTitle}>{title}</h3>}
        {showAIInsights && (
          <button
            className={clsx(styles.aiInsightButton, { [styles.loading]: isLoading })}
            onClick={handleAIInsightClick}
            aria-label="Show AI-powered insights"
          >
            {isLoading ? (
              <span className={styles.loadingSpinner}>⚡</span>
            ) : (
              <span className={styles.aiIcon}>🤖</span>
            )}
            <span className={styles.aiButtonText}>AI Insight</span>
          </button>
        )}
      </div>

      <div className={styles.cardContent}>
        {children}
      </div>

      {showInsight && (
        <div className={styles.aiInsightPanel}>
          <div className={styles.insightHeader}>
            <span className={styles.insightIcon}>💡</span>
            <h4>AI-Powered Insight</h4>
          </div>
          <p className={styles.insightContent}>{aiInsight}</p>
          <div className={styles.insightActions}>
            <button className={styles.insightAction}>Explain Further</button>
            <button className={styles.insightAction}>Show Example</button>
            <button className={styles.insightAction}>Related Concepts</button>
          </div>
        </div>
      )}
    </div>
  );
};

export default AIPoweredCard;
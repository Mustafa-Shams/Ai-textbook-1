import React, { useEffect } from 'react';
import AIPoweredUIPlugin from '../components/AIPoweredUIPlugin';
import ChatWidget from '../components/ChatWidget';

const LayoutEnhancer = ({ children }) => {
  useEffect(() => {
    // Initialize any global AI-powered enhancements
    initializeGlobalEnhancements();
  }, []);

  const initializeGlobalEnhancements = () => {
    // Add global styles for AI-powered elements
    const style = document.createElement('style');
    style.textContent = `
      .ai-tooltip {
        position: absolute;
        top: -40px;
        right: 0;
        background: linear-gradient(135deg, #2563eb, #7c3aed);
        color: white;
        padding: 0.5rem 1rem;
        border-radius: 8px;
        font-size: 0.8rem;
        opacity: 0;
        transition: opacity 0.3s ease;
        pointer-events: none;
        z-index: 1000;
      }

      .ai-suggestion {
        position: absolute;
        right: -8px;
        top: 50%;
        transform: translateY(-50%);
        background: #10b981;
        color: white;
        font-size: 0.6rem;
        padding: 0.2rem 0.5rem;
        border-radius: 10px;
        opacity: 0;
        transition: opacity 0.3s ease;
      }

      .ai-interactive-icon {
        margin-left: 0.5rem;
        opacity: 0;
        transition: opacity 0.3s ease;
        cursor: pointer;
      }

      .ai-explanation-panel {
        position: fixed;
        top: 50%;
        left: 50%;
        transform: translate(-50%, -50%);
        background: white;
        border-radius: 12px;
        box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 10px 10px -5px rgba(0, 0, 0, 0.04);
        width: 90%;
        max-width: 500px;
        z-index: 10000;
        font-family: inherit;
      }

      html[data-theme="dark"] .ai-explanation-panel {
        background: #1e293b;
        color: white;
      }
    `;

    document.head.appendChild(style);
  };

  return (
    <>
      {children}
      <AIPoweredUIPlugin />
      <ChatWidget />
    </>
  );
};

export default LayoutEnhancer;
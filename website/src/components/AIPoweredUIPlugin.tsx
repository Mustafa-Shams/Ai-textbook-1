import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

const AIPoweredUIPlugin = () => {
  const location = useLocation();

  useEffect(() => {
    // Check if we're in the browser before accessing DOM APIs
    if (typeof window === 'undefined') {
      return;
    }

    // Initialize AI-powered UI enhancements
    initializeAIEnhancements();

    // Listen for route changes to enhance new content
    const handleRouteChange = () => {
      setTimeout(() => {
        enhancePageContent();
      }, 100);
    };

    // Add event listeners
    const observer = new MutationObserver(handleRouteChange);
    observer.observe(document.body, { childList: true, subtree: true });

    return () => {
      observer.disconnect();
    };
  }, [location]);

  // Function to initialize AI-powered enhancements
  const initializeAIEnhancements = () => {
    // Add AI-powered tooltips to code blocks
    addCodeBlockTooltips();

    // Enhance navigation with AI-powered suggestions
    enhanceNavigation();

    // Add interactive elements to headings
    addInteractiveHeadings();
  };

  // Function to enhance page content
  const enhancePageContent = () => {
    addCodeBlockTooltips();
    enhanceNavigation();
    addInteractiveHeadings();
  };

  // Add AI-powered tooltips to code blocks
  const addCodeBlockTooltips = () => {
    const codeBlocks = document.querySelectorAll('.prism-code');
    codeBlocks.forEach((block, index) => {
      if (!block.querySelector('.ai-tooltip')) {
        const tooltip = document.createElement('div');
        tooltip.className = 'ai-tooltip';
        tooltip.innerHTML = `
          <div class="ai-tooltip-icon">🤖</div>
          <div class="ai-tooltip-content">
            <strong>AI Analysis:</strong> This code implements a key concept in robotics.
            <button class="ai-explain-btn">Explain Code</button>
          </div>
        `;
        tooltip.style.cssText = `
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
        `;

        const wrapper = document.createElement('div');
        wrapper.style.position = 'relative';
        wrapper.appendChild(block.cloneNode(true));
        wrapper.appendChild(tooltip);

        block.parentNode?.replaceChild(wrapper, block);

        wrapper.addEventListener('mouseenter', () => {
          tooltip.style.opacity = '1';
        });

        wrapper.addEventListener('mouseleave', () => {
          tooltip.style.opacity = '0';
        });
      }
    });
  };

  // Enhance navigation with AI-powered suggestions
  const enhanceNavigation = () => {
    // Add AI-powered "Continue Learning" suggestions
    const sidebarItems = document.querySelectorAll('.menu__list-item');
    sidebarItems.forEach(item => {
      if (!item.querySelector('.ai-suggestion')) {
        const suggestion = document.createElement('div');
        suggestion.className = 'ai-suggestion';
        suggestion.textContent = 'Recommended Next';
        suggestion.style.cssText = `
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
        `;

        item.style.position = 'relative';
        item.appendChild(suggestion);

        item.addEventListener('mouseenter', () => {
          suggestion.style.opacity = '1';
        });

        item.addEventListener('mouseleave', () => {
          suggestion.style.opacity = '0';
        });
      }
    });
  };

  // Add interactive elements to headings
  const addInteractiveHeadings = () => {
    const headings = document.querySelectorAll('h1, h2, h3, h4');
    headings.forEach(heading => {
      if (!heading.querySelector('.ai-interactive-icon')) {
        const icon = document.createElement('span');
        icon.className = 'ai-interactive-icon';
        icon.innerHTML = '💡';
        icon.style.cssText = `
          margin-left: 0.5rem;
          opacity: 0;
          transition: opacity 0.3s ease;
          cursor: pointer;
        `;

        heading.style.position = 'relative';
        heading.appendChild(icon);

        heading.addEventListener('mouseenter', () => {
          icon.style.opacity = '0.7';
        });

        heading.addEventListener('mouseleave', () => {
          icon.style.opacity = '0';
        });

        icon.addEventListener('click', (e) => {
          e.stopPropagation();
          // Show AI-powered explanation panel
          showAIExplanation(heading.textContent || '');
        });
      }
    });
  };

  // Show AI-powered explanation panel
  const showAIExplanation = (content: string) => {
    const panel = document.createElement('div');
    panel.className = 'ai-explanation-panel';
    panel.innerHTML = `
      <div class="ai-panel-header">
        <span class="ai-icon">🤖</span>
        <h3>AI Explanation</h3>
        <button class="ai-close-btn">&times;</button>
      </div>
      <div class="ai-panel-content">
        <p>This concept relates to advanced robotics principles. In embodied AI systems, this element plays a crucial role in...</p>
        <div class="ai-actions">
          <button class="ai-action-btn">Show Example</button>
          <button class="ai-action-btn">Related Topics</button>
          <button class="ai-action-btn">Practice Exercise</button>
        </div>
      </div>
    `;

    panel.style.cssText = `
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
    `;

    document.body.appendChild(panel);

    // Add close functionality
    const closeBtn = panel.querySelector('.ai-close-btn');
    closeBtn?.addEventListener('click', () => {
      document.body.removeChild(panel);
    });

    // Close when clicking outside
    const handleClickOutside = (e: Event) => {
      if (e.target === panel) {
        document.body.removeChild(panel);
        document.removeEventListener('click', handleClickOutside);
      }
    };

    setTimeout(() => {
      document.addEventListener('click', handleClickOutside);
    }, 0);
  };

  return null;
};

export default AIPoweredUIPlugin;
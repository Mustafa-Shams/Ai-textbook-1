import React, { useState, useEffect, useRef } from 'react';
import './ChatWidget.css';

interface Message {
  id: string;
  content: string;
  role: 'user' | 'assistant';
  timestamp: Date;
}

interface ChatWidgetProps {
  backendUrl?: string;
}

const ChatWidget: React.FC<ChatWidgetProps> = ({ backendUrl = 'https://ai-textbook-1-production.up.railway.app' }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [contextText, setContextText] = useState<string | null>(null);
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });
  const [showTooltip, setShowTooltip] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Scroll to bottom of messages when new messages are added
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Listen for text selection and show tooltip - using window.getSelection() as requested
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      if (selection && selection.toString().trim() !== '') {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setTooltipPosition({
          x: rect.left + window.scrollX,
          y: rect.top + window.scrollY - 40
        });

        setContextText(selection.toString().trim());
        setShowTooltip(true);
      } else {
        setShowTooltip(false);
        setContextText(null);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const handleTooltipAskAI = () => {
    if (contextText) {
      // Add the selected text as a user message
      const newMessage: Message = {
        id: Date.now().toString(),
        content: `Explain this: ${contextText}`,
        role: 'user',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, newMessage]);
      setInputValue('');
      setShowTooltip(false);
      setContextText(null);
    }
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() && !contextText) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      content: inputValue.trim() || `Explain: ${contextText}`,
      role: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const response = await fetch(`${backendUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue.trim() || `Explain: ${contextText}`,
          selected_text: contextText || null,
          session_id: 'session-' + Date.now()
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const assistantMessage: Message = {
        id: Date.now().toString(),
        content: data.response,
        role: 'assistant',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = {
        id: Date.now().toString(),
        content: 'Sorry, I encountered an error. Please try again.',
        role: 'assistant',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setShowTooltip(false);
      setContextText(null);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="chat-widget">
      {/* Tooltip for selected text */}
      {showTooltip && contextText && (
        <div
          className="text-selection-tooltip"
          style={{
            position: 'fixed',
            left: `${tooltipPosition.x}px`,
            top: `${tooltipPosition.y}px`,
            zIndex: 10000,
          }}
        >
          <button
            onClick={handleTooltipAskAI}
            className="tooltip-button"
          >
            Ask AI
          </button>
        </div>
      )}

      {/* Chat window toggle button */}
      <button
        className={`chat-toggle-button ${isOpen ? 'open' : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        <span>{isOpen ? '✕' : '🤖 AI Assistant'}</span>
      </button>

      {/* Chat window */}
      {isOpen && (
        <div className="chat-window">
          <div className="chat-header">
            <h3>AI Assistant</h3>
            <button
              className="close-button"
              onClick={() => setIsOpen(false)}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          <div className="chat-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your AI assistant for the Physical AI and Humanoid Robotics textbook.</p>
                <p>You can ask me questions about the content or select text to get explanations.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.role}`}
                >
                  <div className="message-content">
                    {message.content}
                  </div>
                  <div className="message-timestamp">
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="message assistant">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            {/* Using highlighted text as context indicator */}
            {contextText && (
              <div className="context-indicator">
                <span className="context-label">Using highlighted text as context:</span>
                <span className="context-preview">"{contextText.substring(0, 80)}{contextText.length > 80 ? '...' : ''}"</span>
                <button
                  className="clear-context"
                  onClick={() => setContextText(null)}
                >
                  ✕
                </button>
              </div>
            )}
            <div className="input-container">
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder={contextText ? "Ask about the highlighted text..." : "Ask a question about the textbook content..."}
                className="chat-input"
                rows={2}
              />
              <button
                onClick={handleSendMessage}
                disabled={isLoading || (!inputValue.trim() && !contextText)}
                className="send-button"
              >
                Send
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatWidget;
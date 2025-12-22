import React, { useEffect } from 'react';
import ChatWidget from '../components/ChatWidget';

interface Props {
  children: React.ReactNode;
}

export default function Root({ children }: Props): JSX.Element {
  return (
    <>
      {children}
      <ChatWidget backendUrl={process.env.BACKEND_URL || 'https://ai-textbook-1-production.up.railway.app'} />
    </>
  );
}
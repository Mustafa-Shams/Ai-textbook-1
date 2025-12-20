import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import AIPoweredUIPlugin from '../components/AIPoweredUIPlugin';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <AIPoweredUIPlugin />
    </>
  );
}
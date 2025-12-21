import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import LayoutEnhancer from './LayoutEnhancer';

export default function Layout(props) {
  return (
    <LayoutEnhancer>
      <OriginalLayout {...props} />
    </LayoutEnhancer>
  );
}
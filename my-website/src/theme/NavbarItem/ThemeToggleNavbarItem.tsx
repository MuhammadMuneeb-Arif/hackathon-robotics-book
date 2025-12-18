import React from 'react';
import ThemeToggle from '@site/src/components/UIToggle/ThemeToggle';
import type {Props} from '@theme/NavbarItem/ComponentTypes';

const ThemeToggleNavbarItem = (props: Props): JSX.Element => {
  return <ThemeToggle {...props} />;
};

export default ThemeToggleNavbarItem;
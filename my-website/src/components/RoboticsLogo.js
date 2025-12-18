import React from 'react';

const RoboticsLogo = () => {
  return (
    <svg
      width="300"
      height="70"
      viewBox="0 0 300 70"
      xmlns="http://www.w3.org/2000/svg"
    >
      {/* Robot Head Gradient */}
      <defs>
        <linearGradient id="robotGradient" x1="0" y1="0" x2="1" y2="1">
          <stop offset="0%" stopColor="#00c6ff" />
          <stop offset="100%" stopColor="#0072ff" />
        </linearGradient>
      </defs>

      {/* Minimal Robot Head */}
      <g transform="translate(10, 15)">
        <rect x="0" y="0" width="50" height="50" rx="10" fill="url(#robotGradient)" />
        
        {/* Eyes - minimal geometric style */}
        <rect x="12" y="15" width="6" height="6" rx="1" fill="#fff" />
        <rect x="32" y="15" width="6" height="6" rx="1" fill="#fff" />

        {/* Mouth - subtle tech line */}
        <line x1="15" y1="38" x2="35" y2="38" stroke="#ffffff" strokeWidth="2" strokeLinecap="round" />

        {/* Antenna - futuristic line */}
        <line x1="25" y1="0" x2="25" y2="-10" stroke="#0072ff" strokeWidth="2" />
        <circle cx="25" cy="-12" r="2" fill="#0072ff" />
      </g>

      {/* Tech accent lines */}
      <line x1="5" y1="65" x2="295" y2="65" stroke="#00c6ff" strokeWidth="1" strokeDasharray="4 4" />

      {/* Text */}
      <text
        x="70"
        y="40"
        fontFamily="Roboto, Arial, sans-serif"
        fontSize="22"
        fontWeight="900"
        fill="#111"
        letterSpacing="1"
      >
        Robotics Textbook
      </text>

      {/* Small stylized book icon */}
      <g transform="translate(250, 20)">
        <rect x="0" y="0" width="20" height="26" rx="3" fill="#ffcc00" />
        <line x1="0" y1="6" x2="20" y2="6" stroke="#fff" strokeWidth="1.5" />
        <line x1="0" y1="12" x2="20" y2="12" stroke="#fff" strokeWidth="1.5" />
        <line x1="0" y1="18" x2="20" y2="18" stroke="#fff" strokeWidth="1.5" />
      </g>
    </svg>
  );
};

export default RoboticsLogo;

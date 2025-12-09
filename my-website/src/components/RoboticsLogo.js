import React from 'react';

const RoboticsLogo = () => {
  return (
    <svg
      width="200"
      height="50"
      viewBox="0 0 200 50"
      xmlns="http://www.w3.org/2000/svg"
    >
      {/* Simplified robot head */}
      <g transform="translate(5, 10)">
        {/* Robot head outline */}
        <rect x="0" y="0" width="30" height="30" rx="6" fill="#007acc" />

        {/* Eyes */}
        <circle cx="9" cy="13" r="3" fill="#ffffff" />
        <circle cx="21" cy="13" r="3" fill="#ffffff" />

        {/* Mouth/speaker with educational element */}
        <rect x="10" y="22" width="10" height="2" rx="1" fill="#e0f0ff" />

        {/* Antenna */}
        <line x1="15" y1="0" x2="15" y2="-5" stroke="#007acc" strokeWidth="2" />
        <circle cx="15" cy="-6" r="1.5" fill="#007acc" />
      </g>

      {/* Text "Robotics Textbook" */}
      <text
        x="45"
        y="30"
        fontFamily="Arial, sans-serif"
        fontSize="16"
        fontWeight="bold"
        fill="#333"
      >
        Robotics Textbook
      </text>
    </svg>
  );
};

export default RoboticsLogo;
import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';

interface ScrollAnimationProps {
  children: React.ReactNode;
  className?: string;
  animationClass?: string;
  threshold?: number;
}

const ScrollAnimation: React.FC<ScrollAnimationProps> = ({
  children,
  className = '',
  animationClass = 'fade-in-up',
  threshold = 0.1
}) => {
  const [isVisible, setIsVisible] = useState(false);
  const elementRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const element = elementRef.current;
    if (!element) return;

    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
          observer.unobserve(element);
        }
      },
      { threshold }
    );

    observer.observe(element);

    return () => {
      if (element) {
        observer.unobserve(element);
      }
    };
  }, [threshold]);

  return (
    <div
      ref={elementRef}
      className={clsx(
        className,
        isVisible ? animationClass : 'opacity-0'
      )}
    >
      {children}
    </div>
  );
};

export default ScrollAnimation;
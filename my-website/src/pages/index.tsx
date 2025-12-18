import type { ReactNode } from "react";
import clsx from "clsx";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";
import Layout from "@theme/Layout";
import HomepageFeatures from "@site/src/components/HomepageFeatures";
import Heading from "@theme/Heading";

import styles from "./index.module.css";

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx("hero hero--primary", styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className={clsx("hero__title", "fade-in-up", styles.heroTitle)}>
          {siteConfig.title}
        </Heading>
        <p className={clsx("hero__subtitle", "fade-in-up", styles.heroSubtitle)}>
          {siteConfig.tagline}
        </p>
        <div className={clsx(styles.buttons, "fade-in-up")}>
          <Link
            className="button button--secondary button--lg lift-animation"
            to="/docs/intro"
          >
            Start Learning
          </Link>
          <Link
            className="button button--outline button--lg lift-animation"
            to="/docs/module-1"
          >
            Explore Modules
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />"
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}

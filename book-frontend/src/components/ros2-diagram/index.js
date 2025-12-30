import React from 'react';
import styles from './styles.module.css';

/**
 * ROS 2 Architecture Diagram Component
 *
 * A React component that displays an interactive diagram of the ROS 2 architecture,
 * showing the relationship between AI agents, ROS 2 middleware, and robot components.
 */
const Ros2Diagram = ({ title = "ROS 2 Architecture" }) => {
  return (
    <div className={styles.diagramContainer}>
      <h3 className={styles.diagramTitle}>{title}</h3>
      <div className={styles.architectureDiagram}>
        <div className={styles.aiAgent}>
          <h4>AI Agent</h4>
          <div className={styles.node}>AI Controller Node</div>
          <div className={styles.processing}>Processing</div>
          <div className={styles.decision}>Decision Making</div>
        </div>

        <div className={styles.middleware}>
          <h4>ROS 2 Middleware</h4>
          <div className={styles.ddsLayer}>DDS Layer</div>
          <div className={styles.communication}>
            <div className={styles.topic}>Topics</div>
            <div className={styles.service}>Services</div>
            <div className={styles.action}>Actions</div>
          </div>
        </div>

        <div className={styles.robotComponents}>
          <h4>Robot Components</h4>
          <div className={styles.sensors}>Sensors</div>
          <div className={styles.actuators}>Actuators</div>
          <div className={styles.controllers}>Controllers</div>
        </div>
      </div>

      <div className={styles.flowDescription}>
        <p>
          <strong>Flow:</strong> AI Agent → ROS 2 Communication → Robot Components
        </p>
        <p>
          <strong>Communication Types:</strong> Topics (publish/subscribe), Services (request/response), Actions (goal-based)
        </p>
      </div>
    </div>
  );
};

export default Ros2Diagram;
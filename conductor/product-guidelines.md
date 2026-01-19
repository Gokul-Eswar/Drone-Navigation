# Product Guidelines - Universal Drone Navigation System

## Documentation and Code Style
- **Technical Precision:** All code comments and documentation must be precise and focus on the technical details of algorithmic implementation and system architecture.
- **Systematic Structure:** Use clear, descriptive variable and function names. Documentation should be integrated into the codebase wherever possible (e.g., Doxygen for C++, Docstrings for Python).

## Communication Style
- **Professional & Actionable:** Logs and mission reports must prioritize actionable data and use a professional, direct tone.
- **Safety Criticality:** Safety alerts and emergency protocol notifications must use a high-urgency, direct style to ensure immediate operator awareness.

## Architecture and Layout
- **ROS 2 Standards:** Strict adherence to official ROS 2 package standards and naming conventions is required for all components.
- **Modular vs. Monolithic:** While core logic can be centralized for ease of versioning, the system should maintain a modular and decoupled architecture where major components (SLAM engines, Navigation, Safety Monitoring) are developed as distinct, interoperable packages.

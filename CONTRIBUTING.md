# Contributing

Contributions are welcome. Below is a short guide so things stay consistent.

## Setup

- **ROS 2:** Humble  
- **Platform:** Ubuntu 22.04 recommended  
- **Build:** `colcon build --symlink-install`

Install dependencies:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Code style

- **C++:** Follow the style used in `mecanum_control` (ament_cmake).
- **Python:** Roughly PEP 8; nodes use `rclpy`.
- **License:** New files should have the project license at the top:
  - C++: `// Copyright (c) 2025 Tarik Kahraman` and `// SPDX-License-Identifier: MIT`
  - Python: `# Copyright (c) 2025 Tarik Kahraman` and `# SPDX-License-Identifier: MIT`

## Pull requests

1. Fork the repo.
2. Create a branch: `git checkout -b feature/your-feature`.
3. Make your changes and use clear commit messages.
4. Push and open a PR; describe what you changed.

## Tests

CI runs on pushes to `main` and `develop` and on PRs. Locally:

```bash
source install/setup.bash
colcon test --return-code-on-test-failure
```

Gazebo-dependent tests are skipped in CI; run simulation tests on your machine if needed.

## Maintainer

The main author is Tarik Kahraman. In your fork you can change the maintainer email in `package.xml` files to your own.

## Questions

Open an Issue or join the discussion. Thanks.

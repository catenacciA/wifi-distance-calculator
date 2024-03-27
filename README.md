# Distance Measurement from Access Points

## Project Overview

This project is designed to calculate distances from Wi-Fi Access Points (APs) in real time, leveraging signal strength indicators to estimate proximity. Developed in C++, this application runs within a Docker container based on Debian Linux, ensuring a consistent and isolated development and deployment environment.

**Please note**: This application is specifically tailored for Debian-based Linux environments and is optimized for running within a Docker container. It is not intended for cross-platform use.

## Features

- Real-time calculation of distance from Access Points using signal strength.
- Use of libpcap for capturing Wi-Fi packet information.
- Deployment within a Docker container for consistent development and runtime environments.

## Getting Started

These instructions will cover usage information and for the Docker container 

### Prerequisites

- Docker installed on your machine. [Docker Installation Guide](https://docs.docker.com/get-docker/)
- Basic knowledge of Docker and C++ development.

### Installation

1. **Build the Docker Image**

   Navigate to the root directory of the project and build the Docker image with the following command:

   ```sh
   docker build -t ap-distance-calculator .
   ```

2. **Run the Application in a Container**

   After the image is built, start your container with:

   ```sh
   docker run -it --rm ap-distance-calculator
   ```

   The `-it` flag allocates a pseudo-TTY, which simulates a terminal, and `--rm` cleans up the container and removes the file system when the container exits.

### Development

- To develop and test changes within the Docker container, you can mount your project directory into the container:

  ```sh
  docker run -it --rm -v $(pwd):/workspace ap-distance-calculator
  ```

- This mounts the current directory (`$(pwd)`) to `/workspace` in the container. You can then compile and run your application within the container, reflecting any changes made on your host machine.

## Usage

Once inside the container, your application can be compiled and run with commands specific to your project's build system (e.g., using `make`, `cmake`, etc.).

## Contributing

Contributions to this project are welcome. Please consider the following steps:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-fooBar`).
3. Commit your changes (`git commit -am 'Add some fooBar'`).
4. Push to the branch (`git push origin feature-fooBar`).
5. Create a new Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE.md file for details.

## Acknowledgments

- Thanks to all contributors who have helped to improve this application.
- Special thanks to the open-source community for providing the essential libraries and tools.
- 
# Use the official Ubuntu base image
FROM ubuntu:latest 
FROM node:16.9.0


# Set shell to bash (instead of "sh") for Docker RUN.
SHELL ["/bin/bash", "-c"]
# Set shell to bash (instead of "sh") when opening a new terminal.
ENV SHELL=/bin/bash

# Set the PYTHONUNBUFFERED environment variable
ENV PYTHONUNBUFFERED=1

# Set the working directory
WORKDIR /app

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    nodejs \
    npm

# Install http-server globally
RUN npm install -g http-server

# Copy the backend files
COPY crane_control/ /app/crane_control/

# Create a virtual environment
RUN python3 -m venv /app/venv

# Activate the virtual environment and install Python dependencies
RUN /app/venv/bin/pip install --upgrade pip
COPY crane_control/requirements.txt /app/crane_control/requirements.txt
RUN /app/venv/bin/pip install -r /app/crane_control/requirements.txt

# Expose the necessary ports
EXPOSE 8765 8080 3000

# Copy the proxy-server folder
RUN mkdir proxy-server
WORKDIR /app/proxy-server
RUN npm init -y
RUN npm install express http-proxy-middleware
COPY proxy-server/server.js /app/proxy-server/server.js


# Copy the frontend files
WORKDIR /app/crane-frontend
RUN npm init -y
COPY crane-frontend/package*.json /app/crane-frontend/
COPY crane-frontend/tsconfig.json /app/crane-frontend/
COPY crane-frontend/webpack.config.js /app/crane-frontend/
RUN npm install @types/three three urdf-loader webpack webpack-cli
COPY crane-frontend/src/index.ts /app/crane-frontend/src/index.ts
RUN npx webpack

COPY config/public/* /app/config/public


# # Install frontend dependencies
# RUN cd /app/crane-frontend && npm install



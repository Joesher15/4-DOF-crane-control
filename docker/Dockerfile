# Use the official Ubuntu base image
FROM ubuntu:latest

# Set shell to bash (instead of "sh") for Docker RUN.
SHELL ["/bin/bash", "-c"]
# Set shell to bash (instead of "sh") when opening a new terminal.
ENV SHELL=/bin/bash

# Set the working directory
WORKDIR /app

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    nodejs \
    npm

# Copy the backend files
COPY crane_control/ /app/crane_control/

# Create a virtual environment
RUN python3 -m venv /app/venv

# Set the PYTHONUNBUFFERED environment variable
ENV PYTHONUNBUFFERED=1

# Activate the virtual environment and install Python dependencies
RUN /app/venv/bin/pip install --upgrade pip
COPY crane_control/requirements.txt /app/crane_control/requirements.txt
RUN /app/venv/bin/pip install -r /app/crane_control/requirements.txt


# Expose the necessary ports
EXPOSE 8765 3000

# # Copy the frontend files
# COPY crane-frontend/ /app/crane-frontend/

# # Install frontend dependencies
# RUN cd /app/crane-frontend && npm install



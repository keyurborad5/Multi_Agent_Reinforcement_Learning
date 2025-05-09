# # Base image
# FROM ubuntu:22.04

# # Set non-interactive mode for apt
# ENV DEBIAN_FRONTEND=noninteractive

# # Update & install Python + common packages
# RUN apt-get update && apt-get install -y  software-properties-common curl && \
#     add-apt-repository ppa:deadsnakes/ppa && \
#     apt-get update && \
#     apt-get install -y \
#     python3.11 \
#     python3-pip \
#     python3.11-venv \
#     python3.11-distutils \
#     unzip \
#     x11-apps \
#     libgl1-mesa-glx \
#     libglib2.0-0 \
#     libsm6 \
#     libxrender1 \
#     libxext6 \
#     terminator \
#     && apt-get clean \
#     && rm -rf /var/lib/apt/lists/*

# # Set python & pip alias
# # RUN ln -s /usr/bin/python3.10 /usr/bin/python && ln -s /usr/bin/pip3 /usr/bin/pip
# # Install pip for Python 3.11
# RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.11

# # Set Python 3.11 as default
# # RUN ln -s /usr/bin/python3.11 /usr/bin/python && ln -s /usr/local/bin/pip /usr/bin/pip
# RUN ln -sf /usr/bin/python3.11 /usr/bin/python && ln -sf /usr/local/bin/pip /usr/bin/pip


# # Create working directory
# WORKDIR /app

# # Copy requirements and install
# COPY requirements.txt .
# RUN python3.11 -m pip install --upgrade pip && \
#     python3.11 -m pip install --no-cache-dir --root-user-action=ignore -r requirements.txt 

# # Copy zipped application folder
# COPY group13.zip .

# # Unzip application
# RUN unzip group13.zip && rm group13.zip

# ENTRYPOINT ["/bin/bash"]
# # Default command (adjust as needed)
# CMD ["terminator"]
#--------
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install base tools + add Python 3.11 PPA
RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update

# Install Python 3.11 and GUI dependencies
RUN apt-get install -y \
    python3.11 \
    python3.11-venv \
    python3.11-distutils \
    unzip \
    x11-apps \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxrender1 \
    libxext6 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install pip for Python 3.11
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.11

# Set Python 3.11 as default
RUN ln -sf /usr/bin/python3.11 /usr/bin/python && \
    ln -sf /usr/bin/python3.11 /usr/bin/python3 && \
    ln -sf /usr/local/bin/pip /usr/bin/pip

# Now install terminator AFTER python is symlinked
RUN apt-get update && apt-get install -y terminator

# Set working directory
WORKDIR /app

# Copy requirements and install
COPY requirements.txt .
RUN python3.11 -m pip install --upgrade pip && \
    python3.11 -m pip install --no-cache-dir --root-user-action=ignore -r requirements.txt 

# Copy and unzip app
COPY group13.zip .
RUN unzip group13.zip && rm group13.zip

# Set entry
ENTRYPOINT ["/bin/bash"]

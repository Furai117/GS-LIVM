# Use micromamba for efficient conda environment management
FROM mambaorg/micromamba:1.5.7

# Set working directory
WORKDIR /workspace/GS-LIVM

# Copy environment file and create the conda environment
COPY env.yaml /tmp/env.yaml
RUN micromamba env create -f /tmp/env.yaml && \
    micromamba clean --all --yes

# Make the environment active by default
ENV PATH="/opt/conda/envs/gslivm/bin:$PATH"

# Copy project files
COPY . /workspace/GS-LIVM

# Default command
CMD ["bash"]

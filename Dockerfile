FROM debian:bookworm-slim

ARG DEBIAN_FRONTEND="noninteractive"

COPY requirements.txt /tmp/requirements.txt

RUN << EOF
	apt-get update
	apt-get install --yes --no-install-recommends \
		python3-dev python3-pip python3-numpy proj-bin python3-serial
	apt-get install --yes --no-install-recommends \
		python3-lxml python3-future
	python3 -m pip install --break-system-packages -r /tmp/requirements.txt
    rm -rf /var/lib/apt/lists/*
EOF

WORKDIR /app/

COPY antennaTracker.py /app/antennaTracker.py

ENTRYPOINT ["python3", "antennaTracker.py"]

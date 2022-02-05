#!/bin/bash

sudo ln -sf ~/pibot_ros/pibot_upstart/pibotenv /etc/pibotenv
sudo ln -sf ~/pibot_ros/pibot_upstart/pibot_start.sh /usr/bin/pibot_start
sudo ln -sf ~/pibot_ros/pibot_upstart/pibot_stop.sh /usr/bin/pibot_stop
sudo ln -sf ~/pibot_ros/pibot_upstart/pibot_restart.sh /usr/bin/pibot_restart
sudo cp ~/pibot_ros/pibot_upstart/pibot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable pibot
sudo systemctl is-enabled pibot

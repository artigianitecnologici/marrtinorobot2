echo "Install docker.io"
sudo apt -y install docker.io
sudo systemctl status docker
sudo systemctl enable docker
sudo apt -y install docker-compose
echo "--------"
echo "sudo usermod -aG docker ${USER}"
echo "su - ${USER}"
echo "sudo usermod -aG docker marrtino"

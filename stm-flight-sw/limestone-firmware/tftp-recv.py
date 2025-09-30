from tftpy import TftpClient

client = TftpClient('192.168.0.10', 69)
client.download('messages.txt', 'messages.txt', timeout=1000, retries=0)

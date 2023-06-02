import requests

url = 'https://simulator-server.herokuapp.com/simulationVideo'

with open('rps/static/text/Code.txt', 'rb') as f:
    r = requests.post('https://simulator-server.herokuapp.com/simulationVideo', files={'Code.txt': f})


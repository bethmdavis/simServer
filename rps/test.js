
var formdata = new FormData();
formdata.append("code", "rps/static/text/Code.txt", "[PROXY]");

var requestOptions = {
  method: 'POST',
  body: formdata,
  redirect: 'follow'
};

fetch("https://simulator-server.herokuapp.com/simulationVideo", requestOptions)
  .then(response => response.text())
  .then(result => console.log(result))
  .catch(error => console.log('error', error));
const fs = require('fs');
const readline = require('readline');
const http = require('http');
const express = require('express');
const webSocket = require('ws');

const app = express();

app.use(express.static('public'))

const server = http.createServer(app);

const wsServer = new webSocket.Server({server});

wsServer.on('connection', (ws) => {
  console.log("Connection established");

  ws.on('message', (msg) => {
    let filename;
    if(msg === 'kalman') filename = 'kalman.txt';
    else if(msg === 'variacao') filename = 'variacao.txt';
    else filename = 'padrao.txt';
    readFile(ws, filename);
  });
  

});

server.listen(8080, () => {
  console.log("Aplicação aberta em http://localhost:8080");
})

function readFile(ws, filename) {
  /*************************************
   ----File Variables and Functions---- 
  **************************************/

 let flagStart = true;
 let flagBall = true;
 let flagBlue = false;
 let flagYellow = false;

 const readInterface = readline.createInterface({
   input: fs.createReadStream(filename),
   //output: process.stdout,
   console: false
 });

 const data = {
   balls: [],
   blue: [],
   yellow: []
 }

 let balls = []
 let blue = []
 let yellow = []

 readInterface.on('line', function(line) {
   if(!flagStart) {
     if(flagBall) {
       if(line.split(' ')[0] !== 'Blue') {
         const coordinates = line.split(',');
         data.balls.push({
           x: parseFloat(coordinates[1]),
           y: parseFloat(coordinates[2]),
         })
       } else {
         flagBall = false;
         flagBlue = true;
       }
     } else if(flagBlue) {
       if(line.split(' ')[0] !== 'Yellow') {
         const coordinates = line.split(',');
         data.blue.push({
           id: parseInt(coordinates[0]),
           x: parseFloat(coordinates[1]),
           y: parseFloat(coordinates[2]),
         })
       } else {
         flagBlue = false;
         flagYellow = true;
       }
     } else if(flagYellow) {
       if(line.split(' ')[0] !== 'Frame') {
         const coordinates = line.split(',');
         data.yellow.push({
           id: parseInt(coordinates[0]),
           x: parseFloat(coordinates[1]),
           y: parseFloat(coordinates[2]),
         })
       } else {
         flagYellow = false;
         flagBall = true;
         ws.send(JSON.stringify(data));
         //console.log(data);
         data.balls = [];
         data.blue = [];
         data.yellow = [];
       }
     }
   } else {
     flagStart = false;
   }
 });

 readInterface.on('close', () => {
   console.log("Terminou!");
   ws.send("End");
 });
}

// const robots = []

function setup() {
  createCanvas(1300, 750);
  buildField();
}

function draw() {
}

let selectDisplay = true;

let socket;

let valueToSend = "";

let robotsBlue = []
let robotsYellow = []

class Ball {
  constructor(x, y) {
    this.x = x + 650;
    this.y = y + 400;
    if(this.x < 0) this.x = 0;
    else if(this.x > 1300) this.x = 1295;
    if(this.y < 0) this.y = 0;
    else if(this.y > 750) this.y = 745;
  }

  show() {
    fill(255,165,0);
    ellipse(this.x,this.y,12,12);
  }
}

class Robot {
  constructor(x, y, id, color){
    this.x = x + 650;
    this.y = y + 400;
    if(this.x < 0) this.x = 0;
    else if(this.x > 1300) this.x = 1295;
    if(this.y < 0) this.y = 0;
    else if(this.y > 750) this.y = 745;
    this.id = id;
    this.color = color;
  }

  update(x, y) {
    this.x = x + 750;
    this.y = y + 300;
    if(this.x < 0) this.x = 0;
    else if(this.x > 1300) this.x = 1295;
    if(this.y < 0) this.y = 0;
    else if(this.y > 750) this.y = 745;
  }

  show() {
    if(this.color == 'blue') fill(0,0,255);
    else fill(255,255,0);
    ellipse(this.x,this.y,30,30);
    if(this.color == 'blue') fill(255,255,255);
    else fill(0,0,0);
    text(this.id.toString(), this.x - 3, this.y + 3);
  }
}

function buildField() {
  background(14,107,14);
  stroke(255,255,255);
  fill(14,107,14);
  ellipse(width/2, height/2, 80, 80);
  stroke(255,255,255);
  line(width/2, 0, width/2, height);
  fill(14,107,14);
  ellipse(110, height/2, 80, 80);
  ellipse(width - 110, height/2, 80, 80);
  rect(0,height/2 - 120,120,240);
  rect(width - 120,height/2 - 120,121,240);
  rect(0,height/2 - 60,60,120);
  rect(width - 60,height/2 - 60,61,120);
  stroke(0,0,0);
}

window.onload = () => {

  socket = new WebSocket('ws://localhost:8080');

  socket.onmessage = msg => {

    if(msg.data === "End") {
      alert("Fim do Log");
      selectDisplay = true;
      document.getElementById("start-btn").classList.remove("disabled");
      return;
    }

    if(selectDisplay) {
      document.getElementById("start-btn").classList.add("disabled");
      selectDisplay = false;
    }

    const {data} = msg;
    const objData = JSON.parse(data);
    buildField();
  
    for(let i in objData.balls) {
      const ball = new Ball(objData.balls[i].x/8, objData.balls[i].y/8);
      ball.show();
    }
  
    for(let i in objData.blue) {
      robotsBlue.push(new Robot(objData.blue[i].x/8,objData.blue[i].y/8,objData.blue[i].id, 'blue'));
    }
  
    for(let i in objData.yellow) {
      robotsYellow.push(new Robot(objData.yellow[i].x/8,objData.yellow[i].y/8,objData.yellow[i].id, 'yellow'));
    }
  
    for(let i in robotsBlue) {
      robotsBlue[i].show();
    }
  
    for(let i in robotsYellow) {
      robotsYellow[i].show();
    }
  
    robotsBlue = [];
    robotsYellow = []
  };

  const elems = document.querySelectorAll('select');
  const instances = M.FormSelect.init(elems);

  document.getElementById('start-btn').addEventListener('click', () => {
    if(valueToSend !== "") socket.send(valueToSend);
  });

  document.querySelector('select').addEventListener('change', myFunc);

  valueToSend = document.querySelector("select").value;
}

function myFunc(e) {
  valueToSend = document.querySelector("select").value;
}
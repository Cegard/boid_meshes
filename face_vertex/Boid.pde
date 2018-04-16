import java.util.Map;


class Boid {
  Node node;
  int grabsMouseColor;//color
  int avatarColor;
  // fields
  PVector pos, vel, acc, ali, coh, sep; // pos, velocity, and acceleration in
  // a vector datatype
  float neighborhoodRadius; // radius in which it looks for fellow boids
  float maxSpeed = 4; // maximum magnitude for the velocity vector
  float maxSteerForce = .1f; // maximum magnitude of the steering vector
  float sc = 3; // scale factor for the render of the boid
  float flap = 0;
  float t = 0;
  boolean isInImmediate = true;
  PShape modelShape = createShape(GROUP);
  
  Map<PShape, ArrayList<PVector>> faceList = new HashMap();
  Map<PVector, ArrayList<PShape>> vertexList = new HashMap();
  
  PVector vertex_1;
  PVector vertex_2;
  PVector vertex_3;
  PVector vertex_4;
  
  PShape face_1 = createShape();
  PShape face_2 = createShape();
  PShape face_3 = createShape();
  PShape face_4 = createShape();
  
  
  Boid(PVector inPos) {
    modelShape.disableStyle();
    grabsMouseColor = color(0, 0, 255);
    avatarColor = color(255, 0, 0);
    pos = new PVector();
    pos.set(inPos);
    node = new Node(scene) {
      // Note that within visit() geometry is defined at the
      // node local coordinate system.
      @Override
      public void visit() {
        if (animate)
          run(flock);
        render();
      }

      // Behaviour: tapping over a boid will select the node as
      // the eye reference and perform an eye interpolation to it.
      @Override
      public void interact(TapEvent event) {
        if (avatar != this && scene.eye().reference() != this) {
          avatar = this;
          scene.eye().setReference(this);
          scene.interpolateTo(this);
        }
      }
    };
    node.setPosition(new Vector(pos.x, pos.y, pos.z));
    vel = new PVector(random(-1, 1), random(-1, 1), random(1, -1));
    acc = new PVector(0, 0, 0);
    neighborhoodRadius = 100;
    buildStructures();
    buildWholeShape();
  }

  void run(ArrayList bl) {
    t += .1;
    flap = 10 * sin(t);
    // acc.add(steer(new PVector(mouseX,mouseY,300),true));
    // acc.add(new PVector(0,.05,0));
    if (avoidWalls) {
      acc.add(PVector.mult(avoid(new PVector(pos.x, flockHeight, pos.z), true), 5));
      acc.add(PVector.mult(avoid(new PVector(pos.x, 0, pos.z), true), 5));
      acc.add(PVector.mult(avoid(new PVector(flockWidth, pos.y, pos.z), true), 5));
      acc.add(PVector.mult(avoid(new PVector(0, pos.y, pos.z), true), 5));
      acc.add(PVector.mult(avoid(new PVector(pos.x, pos.y, 0), true), 5));
      acc.add(PVector.mult(avoid(new PVector(pos.x, pos.y, flockDepth), true), 5));
    }
    flock(bl);
    move();
    checkBounds();
  }
  
  
  void relateFaceVertices(PShape face, PVector[] vertices){
    
    for (PVector vertex : vertices){
      vertexList.get(vertex).add(face);
      faceList.get(face).add(vertex);
    }
  }
  
  
  void buildStructures(){ 
    vertex_1 = new PVector(3 * sc, 0, 0);
    vertex_2 = new PVector(-3 * sc, 2 * sc, 0);
    vertex_3 = new PVector(-3 * sc, -2 * sc, 0);
    vertex_4 = new PVector(-3 * sc, 0, 2 * sc);
    
    vertexList.put(vertex_1, new ArrayList<PShape>());
    vertexList.put(vertex_2, new ArrayList<PShape>());
    vertexList.put(vertex_3, new ArrayList<PShape>());
    vertexList.put(vertex_4, new ArrayList<PShape>());
    
    faceList.put(face_1, new ArrayList<PVector>());
    faceList.put(face_2, new ArrayList<PVector>());
    faceList.put(face_3, new ArrayList<PVector>());
    faceList.put(face_4, new ArrayList<PVector>());
    
    PVector[] set_1 = {vertex_1, vertex_2, vertex_3};
    PVector[] set_2 = {vertex_1, vertex_2, vertex_4};
    PVector[] set_3 = {vertex_1, vertex_4, vertex_3};
    PVector[] set_4 = {vertex_4, vertex_2, vertex_3};
    
    relateFaceVertices(face_1, set_1);
    relateFaceVertices(face_2, set_2);
    relateFaceVertices(face_3, set_3);
    relateFaceVertices(face_4, set_4);
  }
  
  
  void buildWholeShape(){
    
    for (PShape face : faceList.keySet()){
      PShape singleFace = createShape();
      singleFace.beginShape(TRIANGLE);
      
      for (PVector faceVertex : faceList.get(face))
        singleFace.vertex(faceVertex.x, faceVertex.y, faceVertex.z);
      
      singleFace.endShape();
      modelShape.addChild(singleFace);
    }
  }
  
  
  void immediateMode(){
    beginShape(TRIANGLES);
    
    for (PShape face : faceList.keySet()){
      
      for (PVector faceVertex : faceList.get(face)){
        vertex(faceVertex.x, faceVertex.y, faceVertex.z);
      } 
    }
    
    endShape();
  }
  

  void render() {
    pushStyle();

    // uncomment to draw boid axes
    //scene.drawAxes(10);

    stroke(hue);
    noFill();
    noStroke();
    fill(hue);

    // highlight boids under the mouse
    if (node.track(mouseX, mouseY))
      fill(grabsMouseColor);
    
    if (isInImmediate)
      immediateMode();
    
    else
      shape(modelShape);
    
    popStyle();
  }

  // ///-----------behaviors---------------
  void flock(ArrayList bl) {
    ali = alignment(bl);
    coh = cohesion(bl);
    sep = seperation(bl);
    acc.add(PVector.mult(ali, 1));
    acc.add(PVector.mult(coh, 3));
    acc.add(PVector.mult(sep, 1));
  }

  void scatter() {
  }

  // //------------------------------------

  void move() {
    vel.add(acc); // add acceleration to velocity
    vel.limit(maxSpeed); // make sure the velocity vector magnitude does not
    // exceed maxSpeed
    pos.add(vel); // add velocity to position
    node.setPosition(new Vector(pos.x, pos.y, pos.z));
    node.setRotation(Quaternion.multiply(new Quaternion(new Vector(0, 1, 0), atan2(-vel.z, vel.x)), 
      new Quaternion(new Vector(0, 0, 1), asin(vel.y / vel.mag()))));
    acc.mult(0); // reset acceleration
  }

  void checkBounds() {
    if (pos.x > flockWidth)
      pos.x = 0;
    if (pos.x < 0)
      pos.x = flockWidth;
    if (pos.y > flockHeight)
      pos.y = 0;
    if (pos.y < 0)
      pos.y = flockHeight;
    if (pos.z > flockDepth)
      pos.z = 0;
    if (pos.z < 0)
      pos.z = flockDepth;
  }

  // steering. If arrival==true, the boid slows to meet the target. Credit to
  // Craig Reynolds
  PVector steer(PVector target, boolean arrival) {
    PVector steer = new PVector(); // creates vector for steering
    if (!arrival) {
      steer.set(PVector.sub(target, pos)); // steering vector points
      // towards target (switch target and pos for avoiding)
      steer.limit(maxSteerForce); // limits the steering force to maxSteerForce
    } else {
      PVector targetOffset = PVector.sub(target, pos);
      float distance = targetOffset.mag();
      float rampedSpeed = maxSpeed * (distance / 100);
      float clippedSpeed = min(rampedSpeed, maxSpeed);
      PVector desiredVelocity = PVector.mult(targetOffset, 
        (clippedSpeed / distance));
      steer.set(PVector.sub(desiredVelocity, vel));
    }
    return steer;
  }

  // avoid. If weight == true avoidance vector is larger the closer the boid
  // is to the target
  PVector avoid(PVector target, boolean weight) {
    PVector steer = new PVector(); // creates vector for steering
    steer.set(PVector.sub(pos, target)); // steering vector points away from
    // target
    if (weight)
      steer.mult(1 / sq(PVector.dist(pos, target)));
    // steer.limit(maxSteerForce); //limits the steering force to
    // maxSteerForce
    return steer;
  }

  PVector seperation(ArrayList boids) {
    PVector posSum = new PVector(0, 0, 0);
    PVector repulse;
    for (int i = 0; i < boids.size(); i++) {
      Boid b = (Boid) boids.get(i);
      float d = PVector.dist(pos, b.pos);
      if (d > 0 && d <= neighborhoodRadius) {
        repulse = PVector.sub(pos, b.pos);
        repulse.normalize();
        repulse.div(d);
        posSum.add(repulse);
      }
    }
    return posSum;
  }

  PVector alignment(ArrayList boids) {
    PVector velSum = new PVector(0, 0, 0);
    int count = 0;
    for (int i = 0; i < boids.size(); i++) {
      Boid b = (Boid) boids.get(i);
      float d = PVector.dist(pos, b.pos);
      if (d > 0 && d <= neighborhoodRadius) {
        velSum.add(b.vel);
        count++;
      }
    }
    if (count > 0) {
      velSum.div((float) count);
      velSum.limit(maxSteerForce);
    }
    return velSum;
  }

  PVector cohesion(ArrayList boids) {
    PVector posSum = new PVector(0, 0, 0);
    PVector steer = new PVector(0, 0, 0);
    int count = 0;
    for (int i = 0; i < boids.size(); i++) {
      Boid b = (Boid) boids.get(i);
      float d = dist(pos.x, pos.y, b.pos.x, b.pos.y);
      if (d > 0 && d <= neighborhoodRadius) {
        posSum.add(b.pos);
        count++;
      }
    }
    if (count > 0) {
      posSum.div((float) count);
    }
    steer = PVector.sub(posSum, pos);
    steer.limit(maxSteerForce);
    return steer;
  }
  
  
  void changeMode(){
    isInImmediate = !isInImmediate;
  }
}
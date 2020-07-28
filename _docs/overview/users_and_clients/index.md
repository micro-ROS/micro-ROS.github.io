---
title: Users and Clients
permalink: /docs/overview/users_and_clients/

ofera_consortium:
  - path: images/eProsima.png
    text: Some text 1 ads
  - path: images/Bosch.png
    text: Some text 1 ads
  - path: images/Fiware.png
    text: The FIWARE Foundation is the legal independent body providing shared resources to help achieving the FIWARE mission that is to develop an open sustainable ecosystem around the FIWARE open source platform, created to drive the definition of public, royalty-free and implementation-driven software platform standards that ease the creation of Smart Applications in multiple sectors.
    title: FIWARE
    url: www.fiware.org
  - path: images/PIAP.png
    text: Some text 3

partners_list:
  - path: images/WR.png
    text: Some text 1 ads
  - path: images/Zephyr.jpeg
    text: Some text 2

mw_users:
  - path: images/renesas.png
    text: Some text 1 ads
  - path: images/robotis.png
    text: Some text 2
  - path: images/dronecode.png
    text: Some text 2

use_cases:
  - path: images/Sony.png
    text: Some text 1 ads
  - path: images/3D_Robotics.png
    text: Some text 2
  - path: images/OpenRobotics.png
    text: Some text 3
  - path: images/OpenRobotics.png
    text: Some text 3
  - path: images/OpenRobotics.png
    text: Some text 3
  - path: images/OpenRobotics.png
    text: Some text 3
---

<!-- CSS AND JS -->

<style type="text/css">
.modal {
  display: none; 
  position: fixed;
  z-index: 3; 
  left: 0;
  top: 0;
  width: 100%; 
  height: 100%; 
  overflow: auto; 
  background-color: rgb(0,0,0); 
  background-color: rgba(0,0,0,0.4);
}

.modal-content {
  background-color: #fefefe;
  margin: 15% auto; 
  padding: 20px;
  border: 1px solid #888;
  width: 50%; 
}

.logoImage {
  max-height: 100px;
  max-width: 240px;
}

.photo-gallery{
  padding: 0;
  margin: 0;
  list-style: none;
  display: flex;
  flex-wrap: wrap;
  justify-content: space-around;
}

.flex-item {
  flex: 1 0 33%;
  margin: 5px;
  height: 100px;
  display: flex;
  align-items: center;
  justify-content: center;
}

</style>

<script>
window.onload = () => {
    var modal_close = document.getElementById("myModal-close");
    var modal_content = document.getElementById("myModal-content");
    var modal_title = document.getElementById("myModal-title");
    var modal_url = document.getElementById("myModal-url");
    var modal = document.getElementById("myModal");

    open_modal = (title,url,txt) => {
        modal.style.display = "block";
        modal_content.innerHTML = txt
        modal_title.innerHTML = title
        modal_url.innerHTML = url
        modal_url.href = "http://" + url
    }

    close_modal = () => {
        modal.style.display = "none";
    }

    window.onclick = (event) => {
        if (event.target == modal) {
            close_modal();
        }
    }
}

</script>


<div id="myModal" class="modal">
  <div class="modal-content">
    <span id="myModal-close" class="close" onclick="close_modal()">&times;</span>
    <h2 id="myModal-title"></h2>
    <p><a id="myModal-url" href="" target="_blank" style="margin-bottom: 10px;"></a></p>
    <p id="myModal-content" style="text-align: justify;"></p>
  </div>
</div>

<!-- CONTENT -->
<h2>OFERA Consortium Members</h2>
<div class="photo-gallery">
  {% for image in page.ofera_consortium %}
    <div class="flex-item">
        <img class="logoImage" src="{{ image.path }}" style="cursor:pointer;" alt="{{ image.title}}" onclick="open_modal('{{ image.title }}','{{ image.url }}','{{ image.text }}')"/>
    </div>
  {% endfor %}
</div>


<h2>Partners</h2>
<div class="photo-gallery">
  {% for image in page.partners_list %}
    <div class="flex-item">
        <img class="logoImage" src="{{ image.path }}" style="cursor:pointer;" alt="{{ image.title}}" onclick="open_modal('{{ image.title }}','{{ image.url }}','{{ image.text }}')"/>
    </div>
  {% endfor %}
</div>

<h2>Middleware Users</h2>
<div class="photo-gallery">
  {% for image in page.mw_users %}
    <div class="flex-item">
        <img class="logoImage" src="{{ image.path }}" style="cursor:pointer;" alt="{{ image.title}}" onclick="open_modal('{{ image.title }}','{{ image.url }}','{{ image.text }}')"/>
    </div>
  {% endfor %}
</div>


<h2>Use-cases</h2>
<div class="photo-gallery">
  {% for image in page.use_cases %}
    <div class="flex-item">
        <img class="logoImage" src="{{ image.path }}" style="cursor:pointer;" alt="{{ image.title}}" onclick="open_modal('{{ image.title }}','{{ image.url }}','{{ image.text }}')"/>
    </div>
  {% endfor %}
</div>

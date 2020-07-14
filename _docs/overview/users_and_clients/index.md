---
title: Users and Clients
permalink: /docs/overview/users_and_clients/

partners_list:
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

user_list:
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

</style>

<script>
window.onload = () => {
    var modal_close = document.getElementById("myModal-close");
    var modal_content = document.getElementById("myModal-content");
    var modal = document.getElementById("myModal");

    open_modal = (txt) => {
        modal.style.display = "block";
        modal_content.innerHTML = txt
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
    <p id="myModal-content"></p>
  </div>
</div>

<!-- CONTENT -->
<h2>Partners</h2>
<ul class="photo-gallery" style="column-count: 3; list-style-type: none;">
  {% for image in page.partners_list %}
    <li>
        <img src="{{ image.path }}" style="cursor:pointer;" alt="{{ image.title}}" onclick="open_modal('{{ image.text }}')"/>
    </li>
  {% endfor %}
</ul>


<h2>Users</h2>
<ul class="photo-gallery" style="column-count: 3; list-style-type: none;">
  {% for image in page.partners_list %}
    <li>
        <img src="{{ image.path }}" style="cursor:pointer;" alt="{{ image.title}}" onclick="open_modal('{{ image.text }}')"/>
    </li>
  {% endfor %}
</ul>

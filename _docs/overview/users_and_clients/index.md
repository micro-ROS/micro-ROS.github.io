---
title: Members, Partners and Users
permalink: /docs/overview/users_and_clients/

ofera_consortium:
  - path: images/eProsima.png
    text: eProsima is a SME company focused on networking middleware with special attention to the OMG standard called Data Distribution Service for Real-time systems (DDS). The company develops new features and plugins for DDS, interoperability tools, bridges and personalized networking middleware solutions for its customers. eProsima is active in the sectors of Robotics, Critical Applications and IoT, and it participates in joint research and development projects with both companies and universities.
    title: eProsima
    url: www.eprosima.com
  - path: images/Bosch.png
    text: Bosch is a German multinational engineering and technology company. The company's core operating areas are spread across four business sectors, namely mobility (hardware and software), consumer goods (including household appliances and power tools), industrial technology (including drive and control) and energy and building technology.
    title: Bosch
    url: www.bosch.com
  - path: images/Fiware.png
    text: The FIWARE Foundation is the legal independent body providing shared resources to help achieving the FIWARE mission that is to develop an open sustainable ecosystem around the FIWARE open source platform, created to drive the definition of public, royalty-free and implementation-driven software platform standards that ease the creation of Smart Applications in multiple sectors.
    title: FIWARE
    url: www.fiware.org
  - path: images/PIAP.png
    text: Industrial Research Institute for Automation and Measurements PIAP was established in 1965 as a national institute which basic task is to prepare and implement new technologies, automation systems, production facilities and specialist measuring equipment in various branches of industry.
    title: PIAP
    url: www.piap.pl

partners_list:
  - path: images/WR.png
    text: Wind River is accelerating digital transformation of critical infrastructure by delivering the technology and expertise that enable the deployment of safe, secure, and reliable IoT systems.
    title: Wind River
    url: www.windriver.com
  - path: images/Zephyr.jpeg
    text: The Zephyr Project is a Linux Foundation hosted Collaboration Project. It’s an open source effort uniting developers and users in building a best-in-class small, scalable, real-time operating system (RTOS) optimized for resource-constrained devices, across multiple architectures. As an open source project, the community evolves the project to support new hardware, developer tools, sensors, and device drivers.
    title: Zephyr Project
    url: www.zephyrproject.org

mw_users:
  - path: images/renesas.png
    text: Renesas is a global semiconductor company delivering trusted embedded design innovation with complete semiconductor solutions that enable billions of connected, intelligent devices to enhance the way people work and live.
    title: Renesas
    url: www.renesas.com
  - path: images/robotis.png
    text: SRobotis is a global robot solutions provider and one of the leading manufacturers of robotic hardware. The company is the exclusive producer of the DYNAMIXEL brand of all-in-one smart servos. Robotis specializes in the manufacture of robotic hardware and full robot platforms for use in all fields of study and industry, as well as educational robotics kits for all ages and skill levels.
    title: Robotis
    url: www.robotis.us
  - path: images/dronecode.png
    text: Dronecode is a nonprofit hosted under the Linux Foundation, dedicated to fostering open-source components and their communities. Working with top developers, end-users, and adopting vendors to create opportunities for collaboration.
    title: Dronecode
    url: www.dronecode.org

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

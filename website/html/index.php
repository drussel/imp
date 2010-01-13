<?PHP
   include("../inc/conf.inc.php");
   include("../inc/site.inc.php");
   include("../inc/misc.inc.php");
   $page = $_GET['page'];
   if (!$page) { $page = "home"; }
?>


<?PHP
   if ($page == "home") {
   } elseif ($page == "imp") {
   } elseif ($page == "groups") {
   } elseif ($page == "pubs") {
   } elseif ($page == "doc") {
   } else {
     #ini_set('display_errors',1);
     #error_reporting(E_ALL|E_STRICT);
     header("HTTP/1.1 404 Not Found");
     echo "<!DOCTYPE HTML PUBLIC \"-//IETF//DTD HTML 2.0//EN\">";
     echo "<html><head>";
     echo "<title>404 Not Found</title>";
     echo "</head><body>";
     echo "<h1>Not Found</h1>";
     echo "<p>The requested page ";
     # should be OK since it is just being sent back to the user
     echo $page;
     echo " was not found on this server.</p>";
     echo "<hr>";
     echo "</body></html>";

     exit;
   }
?>

<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.01 Transitional//EN"
        "http://www.w3.org/TR/html4/loose.dtd">

<head>
<title>IMP Community</title>
<meta http-equiv="Content-Type" content="text/html; charset=utf-8">
<meta name="description" content="IMP Community site for the development of an Integrative Modeling Platform">
<meta name="keywords" content="IMP, Integrative, Modeling, Platform">
<link href="imp.css" rel="stylesheet" type="text/css">
<link rel="shortcut icon" type="image/x-icon" href="images/favicon.ico">

<script type="text/javascript"><!--
/* e-mail obfuscation adapted from code by Jason Johnston:
   http://lojjic.net/blog/20030828-142754.rdf.html
*/
function linkEmail() {
  if(!document.getElementsByTagName) return;
  var allElts = document.getElementsByTagName("*");
  if(allElts.length == 0 && document.all) 
    allElts = document.all; //hack for IE5
  for(var i=0; i<allElts.length; i++) {
    var elt = allElts[i];
    var className = elt.className || elt.getAttribute("class") 
      || elt.getAttribute("className");
    if(className && className.match(/\be-mail\b/)
        && elt.firstChild.nodeType == 3) {
      var alls = elt.firstChild.nodeValue;
      fs= alls.split("|")
      addr = fs[0].replace(/\ at\ /i, "@")
        .replace(/\ (dot|period)\ /gi, ".");
      var lnk = document.createElement("a");
      lnk.setAttribute("href","mailto:"+addr);
      lnk.appendChild(document.createTextNode(fs[1]));
      elt.replaceChild(lnk, elt.firstChild);
    }
  }
}
window.onload = linkEmail;
-->
</script>
</head>

<body>


<center>

<div id="impnav">
   <table class="imptnav">
      <tr>
         <td><a href="<? echo $GLOBALS["home"]; ?>">
             <img src="images/imp_logo.png" alt="IMP logo"></a></td>
         <td>
            <div id="implinks">
               <?PHP GetLinks(); ?>
            </div>
         </td>
      </tr>
   </table>
</div>


<div id="impheaderline">
</div>

<div id="container">
  <table class="tcon">
      <tr>
         <td class="twel" colspan="3">
            <?PHP
               if ($page == "home") {
                  echo "<h2>Welcome</h2>
                        <div id=\"goal\">";
                  PrintFile("pages/goal.txt");              
                  echo "</div>";
               } elseif ($page == "imp") {
                  #echo "<h2>Integrative Modeling Platform</h2>
                  echo "<div id=\"imp\">";
                  PrintFile("pages/imp.txt");              
                  echo "</div>"; 
               } elseif ($page == "groups") {
                  echo "<h2>IMP Community groups</h2>
                        <div id=\"groups\">";
                  PrintFile("pages/groups.txt");              
                  echo "</div>"; 
               } elseif ($page == "pubs") {
                  echo "<h2>IMP Publications</h2>
                        <div id=\"pubs\">";
                  GetPublications();              
                  echo "</div>"; 
               } elseif ($page == "doc") {
                  echo "<h2>IMP Documentation</h2>
                        <div id=\"docs\">";
                  GetDocumentation();              
                  echo "</div>"; 
               } else {
                  error_log("Invalid page snuck through"+$page);
               }
            ?>
         </td>
      </tr>
   <?PHP
      if ($page == "home") {
      echo "<tr>
               <td class=\"tnews\">
                  <h3>News</h3>
                  <div class=\"high\">";
                     PrintFile("pages/news.txt");
      echo "      </div>
               </td>
               <td class=\"thres\">
                  <h3>Highlighted research</h3>
                  <div class=\"high\">";
                     PrintFile("pages/hresearch.txt");
      echo "      </div>
               </td>
               <td class=\"thdev\">
                  <h3>Highlighted development</h3>
                  <div class=\"high\">";
                     PrintFile("pages/hdevelopment.txt");
      echo "      </div>
               </td>
            </tr>";
      }
   ?>
   </table>
</div>

<div id="footer">
   <?PHP PrintFile("pages/footer.txt"); ?>
</div>

</center>
</body>
</html>

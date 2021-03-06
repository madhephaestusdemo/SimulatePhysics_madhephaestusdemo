import javax.vecmath.Vector3f;
import com.neuronrobotics.bowlerstudio.threed.*
import org.apache.commons.io.IOUtils;

def base;
//Check if the device already exists in the device Manager
if(args==null){
	base=DeviceManager.getSpecificDevice( "CarlTheWalkingRobot",{
			//If the device does not exist, prompt for the connection
			
			MobileBase m = MobileBaseLoader.fromGit(
				"https://github.com/madhephaestus/carl-the-hexapod.git",
				"CarlTheRobot.xml"
				)
			if(m==null)
				throw new RuntimeException("Arm failed to assemble itself")
			println "Connecting new device robot arm "+m
			return m
		})
}else
	base=args.get(0)


base.DriveArc(new TransformNR(), 0);
PhysicsCore core = PhysicsEngine.get()
PhysicsEngine.clear();

MobileBasePhysicsManager m;

while(MobileBaseCadManager.get( base).getProcesIndictor().get()<1){
	println "Waiting for cad to get to 1:"+MobileBaseCadManager.get( base).getProcesIndictor().get()
	ThreadUtil.wait(1000)
}

HashMap<DHLink, CSG> simplecad = MobileBaseCadManager.getSimplecad(base) 
def baseCad=MobileBaseCadManager.getBaseCad(base)
m = new MobileBasePhysicsManager(base, baseCad, simplecad);

ArrayList<CSG> referencedThingy =  (ArrayList<CSG>)ScriptingEngine
	                    .gitScriptRun(
                                "https://gist.github.com/4814b39ee72e9f590757.git", // git location of the library
	                              "javaCad.groovy" , // file to load
	                              null// no parameters (see next tutorial)
                        )
                     
for(CSG part:referencedThingy){
	  PhysicsEngine.add(new CSGPhysicsManager(
		[part], 
		new Vector3f(6, 2, 180),// starting point
		0.02,// mass
		core
		));                      
}

Thread t =new Thread({
	// walk forward 10 increments of 10 mm totalling 100 mm translation
	TransformNR move = new TransformNR(10,0,0,new RotationNR( 0,0, 2))
	double toSeconds=0.1//100 ms for each increment
	for(int i=0;i<50;i++){
		base.DriveArc(move, toSeconds);
		ThreadUtil.wait((int)toSeconds*1000)
	}
})
t.start()
int msLoopTime =200;
BowlerStudioController.setCsg(PhysicsEngine.getCsgFromEngine());
// run the physics engine for a few cycles
for (int i = 0; i < 35000&& !Thread.interrupted(); i++) {
	long start = System.currentTimeMillis();
	PhysicsEngine.stepMs(msLoopTime);
	long took = (System.currentTimeMillis() - start);
	if (took < msLoopTime){
		ThreadUtil.wait((int) (msLoopTime - took)/4);
		
	}else{
		System.gc()
		println "Real time broken! took "+took
		System.gc()
		if(took>2000)
		 return null;
	}
	//if(i%100==0)
	//	new Thread({System.gc()}).start()
 }

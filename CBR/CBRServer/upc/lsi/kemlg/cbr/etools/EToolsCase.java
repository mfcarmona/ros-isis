package upc.lsi.kemlg.cbr.etools;

import upc.lsi.kemlg.cbr.*;
import upc.lsi.kemlg.cbr.distances.*;
import java.io.BufferedReader;


/**
 * This class adapts the Case class to be used in the A-TEAM Project. <br>
 * It adds a method to parse part of the simulation data files the
 * CBR server receives off-line.
 * @see EToolsCBR
 * @see EToolsCaseDescription
 * @see EToolsCaseSolution
 */
public class EToolsCase extends Case 
                       implements java.io.Serializable
{

   private EToolsCase prevCase; //apunta al cas previ en el temps
   private EToolsCase nextCase; //apunta al cas següent en el temps 

  //====||===============================================================
  //===\||/=============== metodos principales ==========================
  //====\/===============================================================
   

  /**
   * Creates a new case with the given case description and case solution. <br> 
   * <br>
   * {PRE: desc is an instance of CaseDescription,
           sol is an instance of CaseSolution
   * }<br>
   * {POST: an instance of EToolsCase has been created.
   * }<br>
   */
    public EToolsCase(CaseDescription desc, CaseSolution sol)
    {
       super(desc,sol);
	   prevCase = null;
	   nextCase = null;
    }
    
	public EToolsCase getPreviousCase()
	{
	   return prevCase;
	}

	public EToolsCase getNextCase()
	{
	   return nextCase;
	}

    public void setPreviousCase(EToolsCase prev) throws RuntimeException
    {
       if (prev != null)
	   {
	      prevCase = prev;
	   }
	   else throw (new RuntimeException("Null pointer to previous case"));
    }
	   		
    public void setNextCase(EToolsCase next) throws RuntimeException
    {
       if (next != null)
       {
          nextCase = next;
       }
       else throw (new RuntimeException("Null pointer to next case"));
    }


  /**
   * Parses part of the off-line simulation data, getting the information of a case. <br>
   * Part of the information to process is passed through arrays
   * No instance of EToolsCase is needed to call this method, as it is an
   * static method.
   * {PRE: attrTab is the attribute table, the one that has the attribute definitions;
           dist is the distance function that the attributes will use to do
                comparison among themselves;
           arrDesc is an array with the names of the attributes in the
                   order their values will appear
           numVars is the number of attributes that form part of the description
           in is a Java Reader stream connected to a file containing the off-line simulation data
   * }<br>
   * {POST: if the name of all the attributes in arrDesc joined with the values in arrVal 
            match the definition in attrTab then returns an instance of
            EToolsCase with all the information associed. 
   * }<br>
   * <br>
   * @param attrTab the attribute table that contains the attribute definitions
   * @param dist the distance function
   * @param arrDesc the array of attribute names
   * @param arrVal the array of attribute values
   * @param numVars the number of attributes that form the case description
   * @param in the source of cases to be parsed 
   * @return EToolsCase the case obtained.
   */
    public static EToolsCase importFromData(AttributeTable attrTab, 
                                           ComposedDistance dist,
                                           String[] arrDesc,
										   int numVarsDesc,
                                           String[] arrSol,
                                           int numVarsSol,
                                           BufferedReader in)
    {
       EToolsCaseDescription desc;
       EToolsCaseSolution sol;
       EToolsCase res;
	   String line;
	   
	   //en este punto la ultima linea leida fue "CASE"
	   
       desc = EToolsCaseDescription.importFromData(attrTab, dist, 
                                                  arrDesc,
                                                  numVarsDesc, in);

       //en este punto se ha leido INPUT..ENDINPUT, y,
	   //TRAINEE..ENDTRAINEE											
												  
       sol = EToolsCaseSolution.importFromData(attrTab,  
	                                          arrSol,
											  numVarsSol,
											  in);
										
	   //en este punto se ha leido OUTPUTP..ENDOUTPUTP, 
	   //OUTPUTG..ENDOUTPUTG, SOLUTION..ENDSOLUTION
	   
	   // WARNING! por ahora nos saltamos el MOREINFO. 										  

	   try
	    {
	        line = in.readLine();
	       while ((line != null) && (!line.startsWith("ENDCASE")))
	        {
	    	    line = in.readLine();
				//System.out.println(line);
				
	        }
	       if (line == null)
	       {
	          throw new java.io.IOException("INCORRECT format of the data. ENDCASE not found.");
	       }
	    }   
	    catch (java.io.IOException e)
	    {
	       e.printStackTrace(); 
	    }

          
       res = new EToolsCase(desc,sol);
       
       return res;
    } 
    
    /**
   * Parses part of the off-line simulation data, getting the information of a case. <br>
   * Part of the information to process is passed through arrays
   * No instance of EToolsCase is needed to call this method, as it is an
   * static method.
   * {PRE: attrTab is the attribute table, the one that has the attribute definitions;
           dist is the distance function that the attributes will use to do
                comparison among themselves;
           arrDesc is an array with the names of the attributes in the
                   order their values will appear
           numVars is the number of attributes that form part of the description
           in is a Java Reader stream connected to a file containing the off-line simulation data
   * }<br>
   * {POST: if the name of all the attributes in arrDesc joined with the values in arrVal 
            match the definition in attrTab then returns an instance of
            EToolsCase with all the information associed. 
   * }<br>
   * <br>
   * @param attrTab the attribute table that contains the attribute definitions
   * @param dist the distance function
   * @param arrDesc the array of attribute names
   * @param arrVal the array of attribute values
   * @param numVars the number of attributes that form the case description
   * @param in the source of cases to be parsed 
   * @return EToolsCase the case obtained.
   */
    public static EToolsCase importFromData(AttributeTable attrTab, 
                                           ComposedDistance dist,
                                           String[] arrDesc,
										   int numVarsDesc,
                                           String[] arrSol,
                                           int numVarsSol,
                                           BufferedReader in,
                                           boolean debug)
    {
       EToolsCaseDescription desc;
       EToolsCaseSolution sol;
       EToolsCase res;
	   String line;
	   
	   //en este punto la ultima linea leida fue "CASE"
	   
       desc = EToolsCaseDescription.importFromData(attrTab, dist, 
                                                  arrDesc,
                                                  numVarsDesc, in,debug);

       //en este punto se ha leido INPUT..ENDINPUT, y,
	   //TRAINEE..ENDTRAINEE											
												  
       sol = EToolsCaseSolution.importFromData(attrTab,  
	                                          arrSol,
											  numVarsSol,
											  in,debug);
										
	   //en este punto se ha leido OUTPUTP..ENDOUTPUTP, 
	   //OUTPUTG..ENDOUTPUTG, SOLUTION..ENDSOLUTION
	   
	   // WARNING! por ahora nos saltamos el MOREINFO. 										  

	   try
	    {
	        line = in.readLine();
	       while ((line != null) && (!line.startsWith("ENDCASE")))
	        {
	    	    line = in.readLine();
				//System.out.println(line);
				
	        }
	       if (line == null)
	       {
	          throw new java.io.IOException("INCORRECT format of the data. ENDCASE not found.");
	       }
	    }   
	    catch (java.io.IOException e)
	    {
	       e.printStackTrace(); 
	    }

          
       res = new EToolsCase(desc,sol);
       
       return res;
    }
	
	/**
   * Parses part of the off-line simulation data, getting the information of a case. <br>
   * Part of the information to process is passed through arrays
   * No instance of EToolsCase is needed to call this method, as it is an
   * static method.
   * {PRE: attrTab is the attribute table, the one that has the attribute definitions;
           dist is the distance function that the attributes will use to do
                comparison among themselves;
           arrDesc is an array with the names of the attributes in the
                   order their values will appear
           numVars is the number of attributes that form part of the description
           in is a Java Reader stream connected to a file containing the off-line simulation data
   * }<br>
   * {POST: if the name of all the attributes in arrDesc joined with the values in arrVal 
            match the definition in attrTab then returns an instance of
            EToolsCase with all the information associed. 
   * }<br>
   * <br>
   * @param attrTab the attribute table that contains the attribute definitions
   * @param dist the distance function
   * @param arrDesc the array of attribute names
   * @param arrVal the array of attribute values
   * @param numVars the number of attributes that form the case description
   * @param in the source of cases to be parsed 
   * @return EToolsCase the case obtained.
   */
    public static EToolsCase parseFromArray(AttributeTable attrTab, 
                                           ComposedDistance dist,
                                           String[] arrDesc,
                                           float[] arrVal,
                                           int numVars)
    {
       EToolsCaseDescription desc;
       EToolsCaseSolution sol;
       EToolsCase res;
      
       //System.out.println("EToolsCase: he entrado");
  
       //System.out.println("EToolsCase: voy a llamr a parseFromArray");  
       desc = EToolsCaseDescription.parseFromArray(attrTab, dist, 
                                                  arrDesc, arrVal,
                                                  numVars);
       //sol = EToolsCaseSolution.importFromData(in);
       //System.out.println("EToolsCase: he vuelto");

       sol = null;
	      
       res = new EToolsCase(desc,sol);
       
       //System.out.println("EToolsCase: he acabado");

       return res;
    }                                                          
}

import org.omg.sysml.interactive.*;
import java.nio.file.*;
public class ValidateSysML {
  public static void main(String[] a) throws Exception {
    SysMLInteractive s = SysMLInteractive.createInstance();
    s.loadLibrary(a[0]);
    int bad = 0;
    for (int i = 1; i < a.length; i++) {
      String src = Files.readString(Path.of(a[i]));
      SysMLInteractiveResult r = s.process(src);
      System.out.println("== " + a[i] + (r.hasErrors() ? " ERRORS" : " ok") + (r.hasWarnings()? " (warnings)":""));
      String iss = r.formatIssues(); if (iss != null && !iss.isBlank()) System.out.println(iss);
      if (r.getException()!=null) System.out.println(r.formatException());
      if (r.hasErrors()) bad++;
    }
    System.exit(bad == 0 ? 0 : 1);
  }
}

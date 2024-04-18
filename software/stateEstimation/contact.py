def contact_effort_threshold(taus, contact_effort_threshold):
    
    return any([abs(tau) > contact_effort_threshold for tau in taus])

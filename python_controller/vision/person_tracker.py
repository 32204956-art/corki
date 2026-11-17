class TargetSelector:
    """Selection never substitutes another track for the selected ID."""
    @staticmethod
    def candidate(people):
        return people[0].track_id if len(people) == 1 else None

    @staticmethod
    def at_point(people, x, y):
        hits = [p.track_id for p in people if p.bbox[0]<=x<=p.bbox[2] and p.bbox[1]<=y<=p.bbox[3]]
        return hits[0] if len(hits)==1 else None
